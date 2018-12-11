from i2c_PiLib import *
from HSV_CV_lib import *
import datetime

#NOT RUNNING YET

def main():
    #DO_CV
    #GET PSI_CAM BY ADDING PSI_R TO PSI_MAP
    #SEND PSI_INITIAL to Arduino
    #SEND DATA TO ARDUINO
    camera = init_camera(540)
    #imgBGR = cv2.imread('../PennParkImages/curvingRoad.jpg')
    while True:
    #/imageName = "./Images/PiPic" + "%s.jpg" % (datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")) + ".jpg"
        imageName = "PiPic.jpg"
        imgBGR = capture_image(camera, imageName)
        imgGray, imghsv = perform_image_transformations(imgBGR)
        offset, psi_r, chosenLine, edges, mask = get_CV_results(imgBGR, imgGray, imghsv)
        debug_chosen_line(offset, psi_r, chosenLine, imgBGR)
##        plot_data(imgBGR, imghsv, mask, edges)
        offsetCam = numToByteArray(offset)
        psiCam = numToByteArray(psi_r)
        offsetCam.append(ord('O'))
        psiCam.append(ord('H'))
        print("Offset is {}").format(offsetCam)
        print("PsiCam is {}").format(psiCam)

        putByteList(offsetCam) #send the offset
        time.sleep(1)
        putByteList(psiCam) #send the angle 
        time.sleep(3)
        print("Psi Est is: %f", getByteList())

if __name__ == "__main__":
    main()
