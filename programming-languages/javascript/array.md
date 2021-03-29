# Array

```javascript
// return a new array of which 
// every element is the return value of callback
// applied on the old array
Array.prototype.map(callback)

const array1 = [1, 4, 9, 16];
// pass a function to map
const map1 = array1.map(x => x * 2);
console.log(map1);
// expected output: Array [2, 8, 18, 32]
// 注意！ 如果调用 map 的是一个强类型的数组（typed array），那么返回的也会是一个
// 相同类型的强类型数组，因此务必注意传入的回调函数的返回类型，否则可能因为 JS 的
// 类型转换机制导致并非期望的结果！

// return a string joining all elements of the array
// separated by given separator, default to ','
Array.prototype.join(separator)
const uint8 = new Uint8Array([10, 20, 30, 40, 50]);
console.log(uint8.join());
// expected output: "10,20,30,40,50"
console.log(uint8.join(''));
// expected output: "1020304050"

const array1 = [1, 2, 3, 4];
const reducer = (accumulator, currentValue) => accumulator + currentValue;
// 1 + 2 + 3 + 4
console.log(array1.reduce(reducer));
// expected output: 10
```

## Reference

\[1\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global\_Objects/Array/map](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Array/map)

\[2\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global\_Objects/TypedArray/join](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/TypedArray/join)

