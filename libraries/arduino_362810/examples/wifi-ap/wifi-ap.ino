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
 * 4. 在右侧的终端中，输入 `reboot` 命令，重启 ESP32 开发板，等待重启完成。
 * 
 * 5. 重启后的 ESP32 开发板，将自动执行 main.js 文件，一切顺利的话会创建一个名为 `BeShell-APP` 的 热点，密码是 `12345678`。
 * 
 * 6. 在左侧文件面板中，双击 main.js 文件，可以编辑该文件，修改代码后，点击右上角的保存按钮，即可保存到 ESP32 开发板的 / 目录下。
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