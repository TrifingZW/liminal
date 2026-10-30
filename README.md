# liminal

多域端口图载具仿真库。空间刚体动力学由宿主物理后端承担，
本库负责拓扑编排、1D 动力流与力模型。

宿主适配层契约见 [docs/adapter-contract.md](docs/adapter-contract.md)。

## 构建

```sh
cargo build --release   # 产出 staticlib 供宿主链接
cargo test              # 运行单元测试
cargo run               # 运行四轮动力链演示
```
