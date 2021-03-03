# String

```javascript
// substr 方法可以提取子串
str1 = "hello world";
res = str1.substr(3,2); // res = "lo"

// Number 类的 toString 方法可以把数字（以给定基数）转换为字符串
// radix is an integer between 2 and 36 (10 by default)
numObj.toString([radix])
var x = 12;
x.toString(2); // "1100"

// 全局函数 parseInt 会尝试把一个字符串（以给定基数）转换为数字
// 字符串开头的空白字符会被忽略，正负号会被识别
// 遇到无法解析的字符时会从此字符开始截断字符串并返回已解析的值
// 若 radix 参数不合法或未解析出任何值，返回 NaN
// radix 参数被接收时会被转换为整数
// [2, 36] 区间内的整数是 radix 转换后的合法取值
//     （虽然传入浮点 radix 也可以工作，但实在不推荐这么做）
// 特别地，若转换结果为 0, NaN 或 Infinity，会使用缺省策略
// 缺省策略下，如果字符串以 0x 或 0X 开头，将按 16 进制处理；否则按 10 进制处理
// 其他未提及的 radix 取值都是不合法的
// string: the value to parse
// radix[optional]: an integer representing radix of the string
parseInt(string [, radix])

// 按换行分隔文本为行的列表
"a\nb\r\nc".split(/\r?\n/)
```

## Reference

\[1\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global\_Objects/Number/toString](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Number/toString)

\[2\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global\_Objects/parseInt](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/parseInt)

\[3\] [https://stackoverflow.com/questions/21895233/how-in-node-to-split-string-by-newline-n/21896652](https://stackoverflow.com/questions/21895233/how-in-node-to-split-string-by-newline-n/21896652)



