import numpy as np
import cv2


class ImagesTest:

    def __init__(self):
        self.img = np.zeros((640,480,3), np.uint8)
        self.basic_test=self.draw_basic_test()
        self.noise_test=self.draw_noise_test()

    def draw_basic_test(self):

        nb_test=4
        images=[np.copy(self.img) for i in range(nb_test)]
        ref_segments=[]
        dic={}
        
        # Draw a blue rectangle : test1_1
        [cv2.rectangle(images[i],(100,200),(300,500),(255,0,0),thickness=-1) for i in range(nb_test)]        
        
        # Draw red circles with a radius of 20px inner and outer of the rectangle : test1_2 & test1_3
        [cv2.circle(images[i],(x,y),40,(0,0,255),-1) for i,x,y in [(1,150,100),(2,200,350)]]

        # Draw a yellow polygon : test 1_3
        pts = np.array([[50,50],[100,100],[200,100],[20,170]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(images[3],[pts],(0,255,255))
        
        return images


    def draw_noise_test(self):
        nb_test=3
        images=[np.copy(self.img) for i in range(nb_test)]

        # Draw a blue rectangle and 2 red circles with a radius smaller and smaller (40,20,1 px)
        [cv2.rectangle(images[i],(100,200),(300,500),(255,0,0),-1) for i in range(nb_test)]

        [cv2.circle(images[i],(150,100),r,(0,0,255),-1) for i,r in [(0,40),(1,20),(2,1)]]
        [cv2.circle(images[i],(400,450),r,(0,255,255),-1) for i,r in [(0,40),(1,20),(2,1)]]
 
        return images


    def get_pixels_coord(self):
 
        tests=[self.basic_test,self.noise_test]
        
        for i,test in enumerate(tests):
            ref_segments=[]
            for img in test:
                dic={}
                blue_coord=[]
                red_coord=[]
                yellow_coord=[]
                for y in range(img.shape[1]):
                    for x in range(img.shape[0]):
                        if list(img[x][y]) == [255,0,0]: #blue stuffs
                            blue_coord.append((x,y))
                        elif list(img[x][y]) == [0,0,255]: #red stuffs
                            red_coord.append((x,y))
                        elif list(img[x][y]) == [0,255,255]: #yellow stuffs
                            yellow_coord.append((x,y))

                if blue_coord:
                    dic[0]=blue_coord
                if red_coord:
                    dic[1]=red_coord
                if yellow_coord:
                    if not red_coord:
                        dic[1]=yellow_coord
                    else:
                        dic[2]=yellow_coord

                ref_segments.append(dic)
    
            if i==0:
                self.ref_basic=ref_segments
            elif i==1:
                self.ref_noise=ref_segments

      
        
    def write_ref_file(self):

        for k in range(len(self.basic_test)):
            path='DataSet/Reference_seg/basic'+str(k+1)+'.seg'
            f=open(path,'w')
            if k==0:
                f.write('width\t640\nheight\t480\nsegments\t1\n')
            else:
                f.write('width\t640\nheight\t480\nsegments\t2\n')
            for seg in range(len(self.ref_basic[k])):
                for i in range(len(self.ref_basic[k][seg])):
                     f.write(str(seg)+'\t'+str(self.ref_basic[k][seg][i][0])+'\t'+str(self.ref_basic[k][seg][i][1])+'\n')
            f.close()

        for k in range(len(self.noise_test)):
            path='DataSet/Reference_seg/noise'+str(k+1)+'.seg'
            f=open(path,'w')
            f.write('width\t640\nheight\t480\nsegments\t3\n')
            for seg in range(len(self.ref_noise[k])):
                for i in range(len(self.ref_noise[k][seg])):
                    f.write(str(seg)+'\t'+str(self.ref_noise[k][seg][i][0])+'\t'+str(self.ref_noise[k][seg][i][1])+'\n')
            f.close()
      
            


imgs=ImagesTest()
imgs.get_pixels_coord()
imgs.write_ref_file()


'''
# Show the images
cv2.imshow('img',images.basic_test[0])
cv2.waitKey(0)
cv2.destroyAllWindows()
'''
