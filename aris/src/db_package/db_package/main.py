import rclpy as rp
from db_package.db_node import DataBaseNode

def main():
    rp.init(args=None)
    dbNode = DataBaseNode()
    rp.spin(dbNode)
    dbNode.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
    
    # # # 테스트용
    # dbManager = DatabaseManager(
    #     host="localhost",
    #     user="root",
    #     password="amr231218!",
    #     database="ArisTeam5"
    # )

    # try:
    #     dbManager.connect()

    #     # 덤프 데이터 넣기
    #     dbManager.truncateTable()
    #     startDate = datetime.strptime('2024-06-01 00:00:00', '%Y-%m-%d %H:%M:%S')
    #     endDate = datetime.strptime('2024-07-30 23:59:59', '%Y-%m-%d %H:%M:%S')
    #     dbManager.insertDummyData(10000, startDate, endDate)

    # finally:
    #     dbManager.disconnect()
