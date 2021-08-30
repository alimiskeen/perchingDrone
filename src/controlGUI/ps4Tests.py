from pyPS4Controller.controller import Controller


if __name__ == '__main__':
    controller = Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()