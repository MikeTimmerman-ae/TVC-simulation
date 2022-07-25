import GUI


def main():
    import sys
    app = GUI.QtWidgets.QApplication(sys.argv)
    MainWindow = GUI.MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
