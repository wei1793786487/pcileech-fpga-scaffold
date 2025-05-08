# PCILeech FPGA Project

This is an enhanced version of the PCILeech FPGA project with the following new features and improvements:

### Major Updates

1. **Added Captain 100T Script**: Added support for the Captain 100T device to work with PCILeech.
2. **Merged RW1C Handling**: Improved handling of PCI configuration space RW1C (Read-Write-1-Clear) registers, optimizing configuration space read/write operations.

3. **Added MSI and MSIX Interrupt Support**:

   - Added MSI (Message Signaled Interrupts) sending capability
   - Implemented MSIX interrupt support, allowing devices to send MSIX type interrupts
   - Enhanced interrupt handling logic for improved system stability

4. **Added Single-Byte Request Support for Read Engine**:

   - Implemented support for single-byte requests in the read engine
   - Enhanced data granularity for more precise memory operations

5. **Added DNA Functionality**:
   - Implemented device identification through DNA (Device-specific Unique Identifier)

Special thanks to:

- **ekk**
- **ufrisk**

### Usage

Usage is the same as the original PCILeech FPGA project. Please refer to the [original project documentation](https://github.com/ufrisk/pcileech-fpga) for more information.

### Device Flexibility

You can use this project to implement any PCIe device you wish to create. The flexible architecture allows for custom device implementations according to your specific requirements.

# PCILeech FPGA 项目

这是 PCILeech FPGA 项目的改进版本，新增了以下功能和改进：

### 主要更新

1. **添加 Captain 100T 脚本**：新增对 Captain 100T 设备的支持，使其可以与 PCILeech 一起使用。
2. **合并 RW1C**：改进了对 PCI 配置空间 RW1C（Read-Write-1-Clear）寄存器的处理，优化了配置空间的读写操作。

3. **添加 MSI 和 MSIX 中断支持**：

   - 新增了 MSI（Message Signaled Interrupts）发送功能
   - 增加了 MSIX 中断支持，允许设备发送 MSIX 类型的中断

4. **添加读引擎的单字节请求支持**：

   - 实现了读引擎中对单字节请求的支持
   - 增强了数据粒度，实现更精确的内存操作

5. **添加 DNA 功能**：
   - 实现了通过 DNA（设备特定唯一标识符）进行设备识别

### 致谢

特别感谢以下项目和贡献者：

- **ekk**的贡献
- **ufrisk**大哥的贡献

### 使用方法

使用方式与原版 PCILeech FPGA 项目相同，请参考[原项目文档](https://github.com/ufrisk/pcileech-fpga)获取更多信息。

ekknod 的[项目地址](https://github.com/ekknod/pcileech-wifi)

### 设备灵活性

您可以用此项目完成你想实现的任意 PCIe 设备。灵活的架构允许按照您的特定需求进行自定义设备实现。

### 模拟生成8192ee

本项目现已支持模拟生成Realtek 8192ee无线网卡设备。通过此功能，您可以在FPGA上模拟一个完整功能的8192ee无线网卡，用于测试、开发或安全研究。

![8192ee无线网卡](生成的bin(Genrated BIN)/8192ee.png)

#### 使用方法

1. 运行相应的生成脚本以创建8192ee项目
2. 按照标准构建流程编译项目
3. 将生成的比特流文件加载到您的FPGA设备上
4. 系统将识别该设备为Realtek 8192ee无线网卡

添加联系方式 Q 群 1044979352
