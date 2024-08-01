import os
import glob
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory

def load_ui(ui_name):
    packageShareDirectory = get_package_share_directory('store_package')
    uiPath = os.path.join(packageShareDirectory, 'ui/')
    ui_file = glob.glob(os.path.join(uiPath, f"{ui_name}.ui"))[0]
    return uic.loadUiType(ui_file)[0]

loginClass = load_ui("login")
mainClass = load_ui("store")
robotClass = load_ui("robot_manage")
dailySalesClass = load_ui("daily_sales")