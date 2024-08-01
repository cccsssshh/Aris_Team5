import sys
import os
from PyQt5.QtWidgets import QApplication
import rclpy as rp
from store_package.login_page import LoginPage
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/local/opt/qt/plugins'  # 이 경로를 실제 Qt 플러그인 경로로 설정

def main(args=None):
    app = QApplication(sys.argv)
    
    loginWindow = LoginPage()
    loginWindow.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Shutting down")

    rp.shutdown()

if __name__ == "__main__":
    main()
