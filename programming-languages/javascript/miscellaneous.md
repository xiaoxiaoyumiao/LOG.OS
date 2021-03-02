# Miscellaneous

* JS 没有常量的概念
* JS 没有权限（公有和私有）的概念
* JS 没有枚举的概念。不追求安全的情形下用结构体实现即可，否则：
  * [https://stackoverflow.com/questions/287903/what-is-the-preferred-syntax-for-defining-enums-in-javascript](https://stackoverflow.com/questions/287903/what-is-the-preferred-syntax-for-defining-enums-in-javascript)
* 关于在构造器中使用访问器
  * 使用 Object.defineProperties 方法
    * [https://stackoverflow.com/questions/5222209/getter-setter-in-constructor](https://stackoverflow.com/questions/5222209/getter-setter-in-constructor)
* 关于文件命名规范
  * 字母全部小写
  * 不要带空格
  * 用破折号（-）连接单词
  * 库文件可用逗点（.），用于体现版本或从属关系
  * ```text
    vue.min.js
    vue-router.js
    jquery.form.js
    jquery-1.4.2.min.js
    ```
  * ref: [https://developer.aliyun.com/article/202570](https://developer.aliyun.com/article/202570)

