# Miscellaneous

* oneko 命令
  * 可以在桌面生成一只会追逐鼠标的猫。
* df 命令
  * 查看分区总空间和剩余空间
  * df \[path\] 可以查看 path 的空间使用情况
* find 命令
  * -name &lt;pattern&gt; 提供待匹配的文件名 pattern
  * -maxdepth &lt;non-negative-number&gt; 控制搜索的最大深度
  * -mindepth &lt;non-negative-number&gt; 控制搜索的最小深度
  * -exec &lt;commands&gt; ; 对匹配到的每个文件执行 command
    * 执行时将 command 中的 {} 替换为当前匹配到的文件
    * exec 读入 command 内容时到分号为止
    * 为了防止被 shell 替换，一般把 {} 和 ; 保护起来，如 '{}' 和 \;
    * 一个能运作的例子：`find . -maxdepth 1 -name "*.lbl" -exec echo '{}' \;` 查找当前目录下所有后缀为 .lbl 的文件并打印文件名

