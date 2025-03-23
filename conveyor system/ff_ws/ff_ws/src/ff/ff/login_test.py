# test.py
import sys
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from login import LoginDialog

def main():
    app = QApplication(sys.argv)
    
    # 로그인 테스트
    login = LoginDialog()
    if login.exec_() != QDialog.Accepted:
        print("로그인 실패")
        sys.exit()
    print("로그인 성공")
    
    # 메인 윈도우 테스트
    window = QMainWindow()
    window.setWindowTitle('테스트 윈도우')
    window.setGeometry(100, 100, 300, 200)
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()