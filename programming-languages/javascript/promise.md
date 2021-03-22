# Promise

很厉害的机制！有空的时候来整理吧

TODO：整理 Promise 用法

 使用 then 和 catch 可以书写 Promise 链

Promise 封装的函数在 Promise 被构造时就会执行，所以经常需要构造一个会返回 Promise 的函数

当一个 Promise 的 then 方法接收的参数是一个返回 Promise 的函数时，就会在这个 Promise resolve 的时候执行这个函数，构造新的 Promise 并返回

大致上是这样，详细原理还不清楚

## Reference

\[1\] [https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global\_Objects/Promise](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Promise)

