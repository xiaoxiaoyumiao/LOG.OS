# Miscellaneous

* save & load
  * transformers 支持把 tokenizer 和 model 保存到本地以及重新读取。
  * 使用 `save_pretrained(path)` 可以保存，使用 `from_pretrained(path, local_files_only=True)` 可以重新读取。
  * 补充：似乎只有 model 用到 `local_files_only` 参数，它的作用是显式要求（在找不到本地文件时）不要尝试下载模型。
  * ref: [https://huggingface.co/transformers/internal/tokenization\_utils.html\#utilities-for-tokenizers](https://huggingface.co/transformers/internal/tokenization_utils.html#utilities-for-tokenizers)
  * [https://huggingface.co/transformers/main\_classes/model.html\#transformers.PreTrainedModel](https://huggingface.co/transformers/main_classes/model.html#transformers.PreTrainedModel)
* 似乎能将 Bert pooler 排除在训练参数外的代码
  * [https://github.com/huggingface/transformers/blob/b832d5bb8a6dfc5965015b828e577677eace601e/examples/run\_squad.py\#L927](https://github.com/huggingface/transformers/blob/b832d5bb8a6dfc5965015b828e577677eace601e/examples/run_squad.py#L927)
* 
