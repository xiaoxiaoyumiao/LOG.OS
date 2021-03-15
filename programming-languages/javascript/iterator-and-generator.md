# Iterator & Generator

## Iterator

> an iterator is any object which implements the [Iterator protocol](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Iteration_protocols#the_iterator_protocol) by having a `next()` method that returns an object with two properties:
>
> `value`The next value in the iteration sequence.
>
> `done`This is `true` if the last value in the sequence has already been consumed. If `value` is present alongside `done`, it is the iterator's return value.

## Generator

```javascript
// 生成器
function *createIterator() {
    yield 1;
    yield 2;
    yield 3;
}
// 生成器能像正规函数那样被调用，但会返回一个迭代器
let iterator = createIterator();
console.log(iterator.next().value); // 1
console.log(iterator.next().value); // 2
console.log(iterator.next().value); // 3
```

## Reference

\[1\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Iterators\_and\_Generators](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Iterators_and_Generators)

\[2\] [https://www.cnblogs.com/xiaohuochai/p/7253466.html](https://www.cnblogs.com/xiaohuochai/p/7253466.html)

