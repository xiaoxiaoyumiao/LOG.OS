# Tensor

```python
# use torch.tensor(data) to create a torch tensor from python list
data = [[1,2],[3,4]]
x_data = torch.tensor(data)

# use torch.from_numpy(data) to create a torch tensor from numpy array
np_array = np.array(data)
x_np = torch.from_numpy(np_array)

# construct special tensors from a shape
shape = (2,3,)
rand_tensor = torch.rand(shape) # random float in (0,1)
ones_tensor = torch.ones(shape) # all ones
zeros_tensor = torch.zeros(shape) # all zeros

# construct special tensors like a given tensor
x_ones = torch.ones_like(x_data) # all ones
x_rand = torch.rand_like(x_data, dtype=torch.float) # all random and of float type

# get shape, data type and device the tensor is stored on
tensor = torch.rand(3,4)
print(f"Shape of tensor: {tensor.shape}")
print(f"Datatype of tensor: {tensor.dtype}")
print(f"Device tensor is stored on: {tensor.device}")
```

