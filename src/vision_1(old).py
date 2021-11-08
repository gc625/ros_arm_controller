import cv2
import numpy as np
'''
given List[np.array(x,y,z)],
returns normalized vectors from List[i] to List[i+1]
'''
def getNormVecs(nodeCoords):
    dist = []
    vecs = []
    for i in range(len(nodeCoords)-1):
        vecs.append((nodeCoords[i+1]-nodeCoords[i])/np.linalg.norm(nodeCoords[i+1]-nodeCoords[i]))
        # dist.append(np.linalg.norm(nodeCoords[i+1]-nodeCoords[i]))


    return vecs



'''
returns List[np.array(x,y,z)] corresponding to each node

cameras : List[cv_image]
'''
def getCoords(cameras):


    x,y,z = [0]*4,[0]*4,[0]*4
    nodeCoords = []
    print(nodeCoords)
    for n in range(len(cameras)):

        hsv = cv2.cvtColor(cameras[n], cv2.COLOR_BGR2HSV)

        yMask = cv2.inRange(hsv, (15,0,0), (36, 255, 255))
        bMask = cv2.inRange(hsv, (110,50,50), (130, 255, 255))
        gMask = cv2.inRange(hsv, (36,0,0), (70, 255, 255))
        rMask = cv2.inRange(hsv, (0,70,50), (10, 255, 255))

        masks = [gMask,yMask,bMask,rMask]

        centers = []
        for mask in masks:
            r,t = cv2.threshold(mask,120,255,1)
            conts,h = cv2.findContours(t,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            for c in conts:
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

            centers.append((cX,cY))


        newX = centers[0][0]
        newY = centers[0][1]


        for i in range(len(centers)):
            centers[i] = ((centers[i][0]-newX),-1*(centers[i][1]-newY))

        print(centers)
        if(n == 1):
            for i in range(len(centers)):
                y[i],z[i] = centers[i][0],centers[i][1]

            # print(y,z)
        else:

            for i in range(len(centers)):
                z[i] = (z[i] + centers[i][1]) / 2
                x[i] = centers[i][0]


    for i in range(len(x)):
        print(x[i])
        nodeCoords.append(np.array((x[i],y[i],z[i])))

    return nodeCoords



def main():
    cv_image, cv_image2 = cv2.imread("image_copy.png"),cv2.imread("image_copy2.png")
    cameras = [cv_image, cv_image2]

    nodeCoords = getCoords(cameras)
    normVecs = getNormVecs(nodeCoords)

    print(nodeCoords , "\n" , normVecs)


if __name__ == '__main__':
    main()
