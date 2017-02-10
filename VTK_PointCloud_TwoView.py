import vtk
from numpy import random
from VTK_PointCloud import VtkPointCloud

# xmins = [0.0, 0.5, 0.0, 0.5]
# xmaxs = [0.5, 1.0, 0.5, 1.0]
# ymins = [0.0, 0.0, 0.5, 0.5]
# ymaxs = [0.5, 0.5, 1.0, 1.0]
xmins = [0.0, 0.5]
xmaxs = [0.5, 1.0]
ymins = [0.0, 0.0]
ymaxs = [1.0, 1.0]


class VtkTwoViewDisplay:
    def __init__(self, leftPointCloud, rightPointCloud, back_ground_rgb_tuple_3=(0, 0, 0)):
        # Renderer
        self.rendererLeft = vtk.vtkRenderer()
        self.rendererLeft.AddActor(leftPointCloud.vtkActor)
        self.rendererLeft.SetBackground(*back_ground_rgb_tuple_3)
        self.rendererLeft.ResetCamera()
        self.rendererLeft.SetViewport(xmins[0], ymins[0], xmaxs[0], ymaxs[0])
        cameraLeft = vtk.vtkCamera();
        cameraLeft.SetPosition(0, 0, 1)
        cameraLeft.SetFocalPoint(0, 0, 0)
        self.rendererLeft.SetActiveCamera(cameraLeft)

        self.rendererRight = vtk.vtkRenderer()
        self.rendererRight.AddActor(rightPointCloud.vtkActor)
        self.rendererRight.SetBackground(*back_ground_rgb_tuple_3)
        # self.rendererRight.ResetCamera()
        self.rendererRight.SetViewport(xmins[1], ymins[1], xmaxs[1], ymaxs[1])
        self.rendererRight.SetActiveCamera(self.rendererLeft.GetActiveCamera())
        # Render Window
        self.renderWindow = vtk.vtkRenderWindow()
        self.renderWindow.SetSize(1600, 800)
        self.renderWindow.AddRenderer(self.rendererLeft)
        self.renderWindow.AddRenderer(self.rendererRight)
        # Interactor
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)

    def display(self):
        # Begin Interaction
        self.renderWindow.Render()
        self.renderWindowInteractor.Start()
        self.renderWindow.Finalize()
        self.renderWindowInteractor.TerminateApp()
        del self.renderWindow, self.renderWindowInteractor


def main():
    pointCloudLeft = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudLeft.addPoint(point)
    pointCloudLeft.addPoint([0,0,0])

    pointCloudRight = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudRight.addPoint(point, (255, 0, 0))
    pointCloudRight.addPoint([0,0,0])

    twoView = VtkTwoViewDisplay(leftPointCloud=pointCloudLeft, rightPointCloud=pointCloudRight)
    twoView.display()


if __name__ == '__main__':
    main()
