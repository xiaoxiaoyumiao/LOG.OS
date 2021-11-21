# torch.nn

### torch.nn

这个包提供了构建 Module 时所需的各种基本模块。

```python
# 如何产生常用的层

# 2维卷积层，带个卷积核
nn.Conv2d(in_channel,out_channel,(kernel_width,kernel_length))
# in_channel代表输入的channel数（比如RGB图像输入就是3个channel）
# out_channel代表输出的channel数（应该就是卷积核的个数，每个核扫一遍出一个channel）
# kernel_size = (width,length)就是每个卷积核的大小了

torch.nn.Linear(in_features, out_features, bias=True)
'''
Applies a linear transformation to the incoming data: y = xA^T + b
This module supports TensorFloat32.
'''

# drop out
torch.nn.Dropout(p=0.5, inplace=False)
'''
p – probability of an element to be zeroed. Default: 0.5
inplace – If set to True, will do this operation in-place. Default: False
'''

nn.Parameter
是 Tensor 的子类，把 Tensor 封装为一个被认为是模型参数的对象。

Container:

ModuleList(modules = None)
把子模块放在一个列表里面，可以像列表一样访问。

Sequential
可以将一系列 module 的输入输出顺次首尾相接，形成一个链状 Module。
# Example of using Sequential
model = nn.Sequential(
          nn.Conv2d(1,20,5),
          nn.ReLU(),
          nn.Conv2d(20,64,5),
          nn.ReLU()
        )
        
torch.nn.Unflatten(dim, unflattened_size)
'''
Unflattens a tensor dim expanding it to a desired shape. For use with Sequential.

dim specifies the dimension of the input tensor to be unflattened, and it can be either int or str when Tensor or NamedTensor is used, respectively.

unflattened_size is the new shape of the unflattened dimension of the tensor and it can be a tuple of ints or a list of ints or torch.Size for Tensor input; a NamedShape (tuple of (name, size) tuples) for NamedTensor input.
'''
torch.nn.Flatten(start_dim=1, end_dim=-1)
'''
Flattens a contiguous range of dims into a tensor. For use with Sequential.

Shape:
Input: (N, *dims)(N,∗dims)

Output: (N, \prod *dims)(N,∏∗dims) (for the default case).
'''
```

## Reference

\[1\] [https://pytorch.org/docs/stable/nn.html](https://pytorch.org/docs/stable/nn.html)

#### 
