# Miscellaneous

* restrict 关键字
* 关于宏定义中使用的 `do {} while(0)`：
  * 使用 `#define MACRO do { [statements] } while (0)`  来包含一个代码块 `[statements]`，就可以书写形如 `MACRO;` 的语句并安全地展开。

