# login_dialog.py
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox

class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.setStyleSheet("""
            QDialog {
                background-color: #f5f5f5;
            }
            QLineEdit {
                padding: 8px;
                border: 1px solid #ddd;
                border-radius: 4px;
                min-width: 200px;
            }
            QPushButton {
                padding: 8px 16px;
                background-color: #2196F3;
                color: white;
                border-radius: 4px;
                border: none;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QLabel {
                font-size: 14px;
            }
        """)
        
    def init_ui(self):
        self.setWindowTitle('로봇 제어 시스템 로그인')
        self.setFixedSize(400, 200)
        layout = QVBoxLayout()
        
        # Username input
        username_layout = QHBoxLayout()
        username_label = QLabel('아이디:')
        self.username_input = QLineEdit()
        self.username_input.setPlaceholderText("아이디를 입력하세요")
        username_layout.addWidget(username_label)
        username_layout.addWidget(self.username_input)
        
        # Password input
        password_layout = QHBoxLayout()
        password_label = QLabel('비밀번호:')
        self.password_input = QLineEdit()
        self.password_input.setPlaceholderText("비밀번호를 입력하세요")
        self.password_input.setEchoMode(QLineEdit.Password)
        password_layout.addWidget(password_label)
        password_layout.addWidget(self.password_input)
        
        # Login button
        self.login_button = QPushButton('로그인')
        self.login_button.clicked.connect(self.check_login)
        
        # Add spacing
        layout.addSpacing(20)
        layout.addLayout(username_layout)
        layout.addSpacing(10)
        layout.addLayout(password_layout)
        layout.addSpacing(20)
        layout.addWidget(self.login_button)
        layout.addSpacing(20)
        
        self.setLayout(layout)
    
    def check_login(self):
        # 나중에 지울 것
        self.username_input.setText("admin")
        self.password_input.setText("1234")

        username = self.username_input.text()
        password = self.password_input.text()
        if username == "admin" and password == "1234":
            self.accept()
        else:
            QMessageBox.warning(self, '로그인 오류', '잘못된 아이디 또는 비밀번호입니다.')