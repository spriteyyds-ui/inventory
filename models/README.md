# 模型说明

本目录用于存放数字识别 ONNX 模型。

默认模型文件名：`digit_cnn.onnx`

## 使用方式

你可以在其他训练电脑完成模型训练，然后把 ONNX 文件拷贝到本目录，文件名保持为 `digit_cnn.onnx`，或在参数里修改模型路径：

- `number_recognizer_node.onnx_model_path`

## 尺寸一致性

当前工程默认按 `64x64` 输入推理，请确保以下参数与 ONNX 输入尺寸一致：

- `number_recognizer_node.digit_input_size`
- `number_recognizer_node.classifier_input_size`

参数文件位置：

- `config/inventory_system.yaml`

`inventory_system.yaml` 是数字识别节点的唯一参数配置文件。单独启动识别节点和启动完整盘库系统都加载这份配置。
