from PySide2 import QtWidgets, QtCore, QtGui
import pymel.core
from maya import OpenMayaUI as omui
from shiboken2 import wrapInstance
import logging

print("IN SPYDR!")

def getMayaMainWindow():
    '''
    Gets the Maya Main Window to be parented underneath later
    Returns:
        QtWidgets.QMainWindow: The Maya MainWindow
    '''
    # Use the OpenMaya API to get a reference to the Maya Main Window
    pointer = omui.MQtUtil.mainWindow()
    # Use wrapInstance to convert the pointer into a QMainWindow that can be used by Pyside
    if pointer:
        return wrapInstance(long(pointer), QtWidgets.QMainWindow)


class SpydrUI(QtWidgets.QMenu):
    '''
    This class holds the QMenu item that will hold all subsequent tools under SPYDR
    '''
    def __init__(self):
        print("In SPYDR UI")

        # Get the Maya Main Window and the main menu bar at the top without specifying a menu preset
        self.parent = getMayaMainWindow()
        self.maya_menu_bar = self.parent.menuBar()

        # Call the super class for the QMenu init method to specify the Class parent
        super(SpydrUI, self).__init__(parent=self.maya_menu_bar)

        # Check if an existing SPYDR Tools menu item exists and remove it if so
        for i in self.maya_menu_bar.findChildren(QtWidgets.QMenu):
            if i.title() == "SPYDR Tools":
                action = i.menuAction()
                self.maya_menu_bar.removeAction(action)

        # Set the title for the tool menu
        self.setTitle("SPYDR Tools")
        # Add the tool menu to Maya's menu bar
        self.maya_menu_bar.addMenu(self)

        # Add additional menus to the SPYDR dropdown
        self.addMenu("Mattia's Rig Tool")


    def setupMenuBar(self):
        pass


    def addMenuItem(myMenu=None, title='', action=''):
        myMenu.addMenu(title)
        if action:
            myMenu.addAction()


    def addSeparator(myMenu):
        myMenu.addSeparator()