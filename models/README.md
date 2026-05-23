# 数字识别模型说明

本目录存放数字识别运行时 ONNX 模型。默认模型文件为：

```text
models/digit_cnn.onnx
```

## 更换模型

可以在训练电脑完成模型训练后，将导出的 ONNX 文件复制到本目录并保持文件名为 `digit_cnn.onnx`。

如果需要使用其他文件名或路径，请修改：

```text
config/inventory_system.yaml
```

对应参数：

- `number_recognizer_node.onnx_model_path`

## 输入尺寸

当前工程默认按 `64x64` 输入推理。更换模型时，确保 ONNX 输入尺寸和以下参数一致：

- `number_recognizer_node.digit_input_size`
- `number_recognizer_node.classifier_input_size`

单独启动识别节点和启动完整盘库系统都会加载 `config/inventory_system.yaml`。识别调试流程见 [../docs/debug_manual.md](../docs/debug_manual.md)。
