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

## Reference

\[1\] [https://pytorch.org/docs/stable/tensors.html](https://pytorch.org/docs/stable/tensors.html)

\[2\] [https://pytorch.org/tutorials/beginner/basics/optimization\_tutorial.html](https://pytorch.org/tutorials/beginner/basics/optimization_tutorial.html)

\[3\] [https://pytorch.org/docs/stable/generated/torch.nn.Module.html\#torch.nn.Module.train](https://pytorch.org/docs/stable/generated/torch.nn.Module.html#torch.nn.Module.train)

