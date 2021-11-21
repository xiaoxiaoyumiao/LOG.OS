# Training

简单来说，当一个 `requires_grad` 为 True ，也就是需要计算梯度的 tensor 的 `backward` 方法被调用时，它会对其所在的计算图执行梯度计算和传播。计算所得的梯度数据会累加在每个 tensor 的 grad 成员变量中。注意在使用时要视情况将 tensor 的梯度清零（ `zero_grad` ）。

`torch.optim` 提供了梯度下降算法的简洁封装。使用 Optimizer 对象可以方便地管理一大批参数的梯度。特别是在整个网络已经封装在一个或若干个 Module 对象里时，使用 `parameters()` 方法就可以获取到一个网络的所有可学习（也就是需要计算梯度）的参数。把所有需要学习的参数传递给 Optimizer 的构造函数即可。常用的优化算法被实现为了它的各个子类。

```python
optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.9)
optimizer = optim.Adam([var1, var2], lr=0.0001)
```

Optimizer 还可以为不同的模型参数设置不同的算法超参数。

```python
optim.SGD([
                {'params': model.base.parameters()},
                {'params': model.classifier.parameters(), 'lr': 1e-3}
            ], lr=1e-2, momentum=0.9)
            
```

以上优化器将对 base 的参数使用 1e-2 的学习率，而对 classifier 的参数使用 1e-3 的学习率。

在训练和测试时，部分模块的行为是不一样的（例如 batch norm 和 dropout）。为了控制这些模块，在训练前需要把模型通过 `train()` 方法置为训练模式。在测试时则需要调用 `eval()` 或 `train(False)` 。（它们都是 Module 的成员方法）

```python
def train(dataloader, model, loss_fn, optimizer):
    size = len(dataloader.dataset)
    for batch, (X, y) in enumerate(dataloader):
        X, y = X.to(device), y.to(device)

        # Compute prediction error
        pred = model(X)
        loss = loss_fn(pred, y)

        # Backpropagation
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if batch % 100 == 0:
            loss, current = loss.item(), batch * len(X)
            print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")
```

## 关于 update cycle 的实现

有时希望使用一个较大的 batch 进行训练，但显存不支持对这么大的 batch 进行前向计算。由于 torch 解耦了梯度计算和梯度更新，可以通过将一个 batch 拆分成多个 mini-batch，在对多个 mini-batch 做梯度传播后再做一次优化器迭代（构成一个 update cycle）。梯度存储可能导致一些额外的显存开销，但和将整个 batch 的数据装入显存相比还是划算的。整体来说是时间换空间。以下代码是一个示例，通过控制参数初始化策略、随即数种子等不变，修改 update cycle 和 batch size 可以看到，这种实现和直接使用大 batch 计算的结果是一致的（可能存在浮点运算带来的误差，可忽略不计）：

```python
class FFNN(torch.nn.Module):
    def __init__(self, input_size, num_hidden_layers, 
            hidden_size, output_size, dropout):
        """
            :param input_size: input size
            :param num_hidden_layers: count of hidden layers. 0 for MLP.
        """
        super().__init__()
        ffnn_layer_list = []        
        current_input_size = input_size        
        for i in range(num_hidden_layers):
            ffnn_layer_list.append(torch.nn.Linear(current_input_size, hidden_size, bias=True))
            ffnn_layer_list.append(torch.nn.ReLU())
            if dropout is not None:
                ffnn_layer_list.append(torch.nn.Dropout(dropout))
            current_input_size = hidden_size
        ffnn_layer_list.append(torch.nn.Linear(current_input_size, output_size, bias=True))

        self.ffnn = torch.nn.Sequential(*ffnn_layer_list)

    def forward(self, x):
        return self.ffnn(x)
        
seed = 42
torch.manual_seed(seed)

data_size = 64
batch_size  = 8
update_cycle = 1
labels = [torch.tensor([x], dtype=torch.float32) for x in range(1,data_size)]
examples = [ torch.cat([torch.tensor([0], dtype=torch.float32), torch.log(x), x*5]) for x in labels]
print(labels[0].dtype, examples[0].dtype)
class SimpleDataset(Dataset):
    def __init__(self, data):
        self.data = data
        
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, index):
        return self.data[index]

# LABEL = [ batch_size, input_width, coef_width ] * coefs[coef_width, label_width]
class SimpleModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.param = FFNN(3, 2, 5, 1, None)
        
    def forward(self, x):
        return self.param(x)

def loss_fn(output, label):
    return torch.mean((label - output)**2, dim=0)

dataset = SimpleDataset(list(zip(examples, labels)))
dataloader = DataLoader(dataset, batch_size=batch_size)
model = SimpleModel()
for tensor in model.parameters():
    torch.nn.init.uniform_(tensor)
model.train()
counter = 0
optimizer = optim.AdamW(model.parameters())
optimizer.zero_grad()
for epoch in range(10):
    for input, label in dataloader:
        output = model(input)
        loss = loss_fn(output, label)
        loss = loss / update_cycle
        loss.backward()
        print(output)
        counter += 1
        if counter % update_cycle == 0:
            
            optimizer.step()
            optimizer.zero_grad()
```

## Reference

\[1\] [https://pytorch.org/docs/stable/tensors.html](https://pytorch.org/docs/stable/tensors.html)

\[2\] [https://pytorch.org/tutorials/beginner/basics/optimization\_tutorial.html](https://pytorch.org/tutorials/beginner/basics/optimization_tutorial.html)

\[3\] [https://pytorch.org/docs/stable/generated/torch.nn.Module.html\#torch.nn.Module.train](https://pytorch.org/docs/stable/generated/torch.nn.Module.html#torch.nn.Module.train)

\[4\] [https://discuss.pytorch.org/t/multiple-forward-before-backward-call/20893](https://discuss.pytorch.org/t/multiple-forward-before-backward-call/20893)

\[5\] [https://medium.com/huggingface/training-larger-batches-practical-tips-on-1-gpu-multi-gpu-distributed-setups-ec88c3e51255](https://medium.com/huggingface/training-larger-batches-practical-tips-on-1-gpu-multi-gpu-distributed-setups-ec88c3e51255)

