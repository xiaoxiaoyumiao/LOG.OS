# Heap

## heapq

```python
import heapq

# heapq 提供一组方法，用于实现堆操作
# 空列表被 heapq 方法作用后转化为堆
# 将列表 x 转化为堆
heapq.heapify(x)
# 将元素 item 压入堆 heap 中
heapq.heappush(heap, item)
# 从堆 heap 中获取最小元素
heapq.heappop(heap)
h = []
for value in iterable:
    heappush(h, value)
```

## Reference

\[1] [https://docs.python.org/3/library/heapq.html](https://docs.python.org/3/library/heapq.html)
