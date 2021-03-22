# Virtual Environment

## Dependency

### Human

* [Virtual Environment](../python-tools/virtual-environment.md)

## Setup, Use & Delete

```bash
# create a virtual environment with name <env_name>
conda create --name <env_name>
conda create -n <env_name>

# specify python version:
conda create -n <env_name> python=<version>
```

使用 `conda activate <env_name>` 激活这个虚拟环境。使用 `conda deactivate` 退出当前激活的虚拟环境。

与 `virtualenv` 不同，conda 并不默认在本地创建虚拟环境的文件夹。使用 `conda info --envs` 查看当前安装的所有包，例如：

```bash
conda info --envs
.../Anaconda3
.../Anaconda3/envs/env
```

删除一个虚拟环境可以使用的命令：

```bash
conda env remove --name <env_name>
conda remove --name <env_name> --all
```

使用 `--prefix` 指定虚拟环境文件夹路径。例如：

```text
conda create --prefix ./envs
```

这个虚拟环境需要通过路径指定激活：

```text
conda activate ./env
```

## Reference

[https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html)

