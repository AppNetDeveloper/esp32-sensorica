/**
 * BeShell 文档主页： https://beshell.become.cool
 * 
 * 编译前准备：
 * 
 * * arduion 菜单栏：`工具` -> `开发板: "xxxxxx"` 选择和你使用的开发板一致。
 * 
 * * arduion 菜单栏：`工具` -> `Partitions Scheme: "xxxxxx"` -> `Default 4MB with ffat (1.2MB APP/1.5MB FATFS)`
 * 
 * 编译上传后：
 *  
 * 1. 通过 USB 连接 ESP32 开发板。
 * 
 * 2. 启动浏览器，打开在线IDE https://beconsole.become.cool，连接 ESP32 开发板。
 * 
 * 3. 在左侧的文件面板中，右键 "/" 目录，选择 `上传文件` ，将 main.js 文件上传到 ESP32 开发板的 / 目录下。
 * 
 * 4. 在文件面板中，双击 main.js 文件，在WEB编辑器中打开该文件，修改 `your SSID` 和 `your PASSWORD`，点击右上角的保存按钮。
 * 
 * 5. 在右侧的终端中，输入 `reboot` 命令，重启 ESP32 开发板，等待重启完成。
 * 
 * 6. 重启后的 ESP32 开发板，将自动执行 main.js 文件，一切顺利的话会连接到你的 WiFi 网络，并输出通过 DHCP 获取的 IP 地址。
 * 
 * 
 */


#include <BeShell.hpp>

be::BeShell beshell ;

void setup() {  
  
  // 应用 WiFi 模块
  beshell.use<be::WiFi>() ;

  // 应用 FS 模块
  beshell.use<be::FS>() ;

  // 将 flash 上的名为 ffat 的分区挂载到 / 目录
  be::FS::mount("/", new be::FatFS("ffat",true)) ;

  // 启动 BeShell
  beshell.setup() ;

  // 运行 /main.js
  beshell.engine->evalScript("/main.js") ;
}

// 主循环
void loop() {
  beshell.loop() ;
}