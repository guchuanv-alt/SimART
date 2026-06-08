# SimART LangChain Material Agent

这是一个面向新手的最小 LangChain agent 原型。它先解决一个很小但真实的问题：

> 用户用自然语言描述需求，agent 调用工具查看/修改 Sionna RT 场景 XML 里的材质。

## 你先理解这三个概念

1. **LLM**
   大模型本身只会读文字、写文字、做判断。

2. **Tool**
   工具是普通 Python 函数。比如 `list_materials()`、`replace_material_refs()`。
   大模型不能直接乱改文件，它只能请求调用这些工具。

3. **Agent**
   Agent = 大模型 + 工具 + 系统提示词 + 执行循环。
   它会自己判断什么时候调用哪个工具。

LangChain 官方文档里也把 agent 描述成“模型循环调用工具直到任务完成”的结构。

## 安装依赖

建议在你的 Sionna/ROS Python 环境之外单独建一个轻量环境学习 agent：

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART
python3 -m venv .venv-agent
source .venv-agent/bin/activate
pip install -r agent/requirements.txt
```

然后设置 API key：

```bash
export OPENAI_API_KEY="你的 key"
```

## 第一步：只用工具，不用大模型

先确认 XML 工具能正常工作：

```bash
python -m agent.sionna_xml_tools \
  --scene SimART_sample_maps/Paris_Small/Paris_Small_sionna/Paris_Small.xml \
  summary
```

这一步很重要：做 agent 前，先保证工具本身可靠。

## 第二步：用 agent 只做 dry-run

默认 `apply_changes=false`，不会真的改文件：

```bash
python -m agent.material_agent \
  "查看 Paris_Small 场景有哪些材质，然后把 concrete 变成红色，但先不要真的修改"
```

## 第三步：允许 agent 修改 XML

```bash
python -m agent.material_agent \
  --apply \
  "把 Paris_Small 场景里 concrete 的显示颜色改成红色，marble 改成绿色"
```

## 当前限制

- 这个原型只处理 XML 材质定义和 `<ref id=\"mat-...\" name=\"bsdf\"/>` 引用。
- 它还没有接 SimART 主 GUI。
- 它还没有自动启动 ROS 仿真。
- 它故意把写文件权限做成 `--apply`，避免新手阶段误改大量文件。

后续可以继续扩展：

- 修改 `.agcfg` 里的 RT 参数，例如 `max_depth`、`specular_reflection`。
- 接入 Sionna preview 启动工具。
- 加 human-in-the-loop，让用户确认后再写文件。
- 用 LangGraph 把“分析需求 -> 修改配置 -> 启动仿真 -> 分析结果”做成多节点工作流。
