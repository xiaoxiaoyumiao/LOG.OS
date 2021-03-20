# Miscellaneous

* Why do we use %% before temp variables in a batch file?\( ref: \[1\] \)
  * In a batch file, the following parsing process is carried out before script execution:
    * %1, %2, ..., %9 are interpreted into command line parameters;
    * %0 is replaced by the command used to execute the batch file;
    * single % is replaced by null;
    * A pattern like %VAR%\(VAR can be any string\) is replaced by corresponding environment variable with name VAR. If VAR doesn't exist, the pattern is replaced by null.
    * %% is interpreted into %.
* where：类似 linux 中的 which 工具
  * `where.exe python` 会给出 python 的位置。
  * .exe 是必要的。

## Reference

[\[1\] Q75634: Percent Signs Stripped from Batch File Text](https://jeffpar.github.io/kbarchive/kb/075/Q75634/)

[\[2\] What is Windows' equivalent of the “which” command in Unix? Is there an equivalent PowerShell command?](https://superuser.com/questions/207707/what-is-windows-equivalent-of-the-which-command-in-unix-is-there-an-equivale)

