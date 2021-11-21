# Miscellaneous

* `np.stack(arrays, axis=0, out=None)` ：沿某个维度将一个 array 的列表连接成一个 array
  * ref: [https://numpy.org/doc/stable/reference/generated/numpy.stack.html](https://numpy.org/doc/stable/reference/generated/numpy.stack.html)
* 将一个数组通过插值对齐到另一个长度不同的数组
  * 代码如下
  * ref: [https://stackoverflow.com/questions/38064697/interpolating-a-numpy-array-to-fit-another-array](https://stackoverflow.com/questions/38064697/interpolating-a-numpy-array-to-fit-another-array)

```python
def align_with_interp(arr_ref, arr):
    arr_interp = interp.interp1d(np.arange(arr.size),arr)
    return arr_interp(np.linspace(0,arr.size-1,arr_ref.size))
```

* 将数组中的 NaN 通过插值替换
  * 代码如下
  * ref：[https://stackoverflow.com/questions/6518811/interpolate-nan-values-in-a-numpy-array](https://stackoverflow.com/questions/6518811/interpolate-nan-values-in-a-numpy-array)

```python
def nan_helper(y):
    return np.isnan(y), lambda z: z.nonzero()[0]

y= np.array([1, 1, 1, np.NaN, np.NaN, 2, 2, np.NaN, 0])
nans, x= nan_helper(y)
y[nans]= np.interp(x(nans), x(~nans), y[~nans])
```

