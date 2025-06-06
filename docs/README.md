# CarManagement
汽车健康检测系统

需求：
本项目旨在开发一个嵌入式系统，通过访问汽车 OBD 接口和 GPS 定位器获取相关数据，并将这些数据上传至网页进行展示。同时，在网页上添加交互功能，实现对模拟的车门锁、车内灯光和空调等设备的控制。此外，尝试调用 AI 接口对汽车数据进行评级并给出保养建议，以提升用户对汽车状态的了解和管理能力。

步骤：
1.首先，需要购买项目所需的各类模块，包括STM32F407ZGT6开发板、CAN 收发器、GPS 模块、ESP32 开发板、舵机、LED 灯、电机等。同时，下载相应的开发软件，如 Arduino IDE 用于 ESP32 的编程开发，以及相关的 CAN 总线调试软件。将硬件模块进行合理连接和布局，完成硬件平台的搭建。在软件方面，配置好开发环境，确保各个模块能够正常通信和工作。
2.CAN 总线作为汽车电子系统中广泛应用的通信协议，了解其工作原理是获取汽车 OBD 数据的基础。深入学习 CAN 总线帧的结构，包括帧起始、仲裁段、控制段、数据段、CRC 段、ACK 段和帧结束等部分。掌握不同类型的 CAN 帧，如数据帧、远程帧、错误帧和过载帧的特点和用途，便于进行CAN总线的使用。
3.nuelen 的 OBD 模拟器是用于模拟汽车 OBD 接口数据的工具，通过它可以在没有实际汽车的情况下进行数据采集和测试。学习该模拟器的使用方法，包括如何连接到开发板、如何配置模拟参数、如何启动和停止模拟等。通过实际操作，熟悉模拟器所提供的各种汽车数据，如发动机转速、车速、冷却液温度等，为后续的数据采集工作做好准备。
4.利用 CAN 收发器与 OBD 模拟器进行连接，通过 CAN 总线协议获取汽车的相关数据。编写程序，解析接收到的 CAN 帧，提取出所需的汽车数据。
5.连接 GPS 模块到开发板，通过USART接收 GPS 模块发送的定位信息。解析 GPS 数据，提取出汽车的经度、纬度、速度等位置信息。对获取到的位置信息进行处理，如坐标转换、数据格式化等，以便后续的上传和显示。
6.将位置信息和汽车数据打包通过USART发送到ESP32。
7.用ESP32组网以及HTML、CSS、JavaScrpit技术搭建网站，并将汽车信息和位置信息打包成JSON字符串上传至网站显示（尝试将网页部署在云服务器上，跳出局域网的限制）。
8.利用舵机模拟车门锁，LED灯模拟车内灯光，电机带动风扇模拟车内空调（转速可调），在网页添加交互按钮进行控制。
9.调用AI接口对汽车数据进行评级，并给出一定的保养建议（尽量完成）。
