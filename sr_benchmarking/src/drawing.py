import numpy as np
import Image, ImageDraw


class ImagesTest:

    def __init__(self):
        # load the test images (PIL format)
        self.basic_test=self.draw_basic_test()
        self.noise_test=self.draw_noise_test()

        self.ref_basic=[]
        self.ref_noise=[]

        # save images as numpy format
        self.basic_images=PIL2array(self.basic_test)
        self.noise_images=PIL2array(self.noise_test)


    def draw_basic_test(self):
        nb_test=4
        images=[Image.new("RGB",(640,480),"black") for i in range(nb_test)]
         
        # Draw a blue rectangle : test1_1
        draw=ImageDraw.Draw(images[0])
        draw.rectangle([(500,300),(200,200)],fill=(0,0,255)) 

        # Draw red circles with a radius of 30px outer of the rectangle : test1_2 
        r=30
        x,y=100,150
        draw=ImageDraw.Draw(images[1])
        draw.rectangle([(500,300),(200,100)],fill=(0,0,255)) 
        draw.ellipse((x-r,y-r,x+r,y+r),fill=(255,0,0)) 

        # Draw red circles with a radius of 30px inner  of the rectangle : test1_3
        r=30
        x,y=400,200
        draw=ImageDraw.Draw(images[2])
        draw.rectangle([(500,300),(200,100)],fill=(0,0,255)) 
        draw.ellipse((x-r,y-r,x+r,y+r),fill=(255,0,0))

        # Draw a yellow polygon outer of the rectangle : test 1_4
        draw=ImageDraw.Draw(images[3])
        draw.rectangle([(500,300),(200,200)],fill=(0,0,255)) 
        draw.polygon([(50,50),(100,100),(200,100),(20,170)],fill=(255,255,0))

        return images


    def draw_noise_test(self):
        nb_test=3
        images=[Image.new("RGB",(640,480),"black") for i in range(nb_test)]

        ### Draw a blue rectangle and 2 red circles with a radius smaller and smaller (40,20,1 px)
        # Image 1 
        draw=ImageDraw.Draw(images[0])
        draw.rectangle([(500,300),(200,100)],fill=(0,0,255)) 
        r=30
        [draw.ellipse((x-r,y-r,x+r,y+r),fill=(255,0,0)) for x,y in [(100,150),(400,200)]]

        # Image 2
        draw=ImageDraw.Draw(images[1])
        draw.rectangle([(500,300),(200,100)],fill=(0,0,255)) 
        r=20
        [draw.ellipse((x-r,y-r,x+r,y+r),fill=(255,0,0)) for x,y in [(150,100),(400,400)]]

        #Image 3

        draw=ImageDraw.Draw(images[2])
        draw.rectangle([(500,300),(200,200)],fill=(0,0,255)) 
        r=1
        [draw.ellipse((x-r,y-r,x+r,y+r),fill=(255,0,0)) for x,y in [(150,100),(400,400)]]

        
        return images


    def get_pixels_coord(self):
 
        tests=[self.basic_test,self.noise_test]
        
        for i,test in enumerate(tests):
            ref_segments=[]
            for img in test:
                width=img.size[0]
                height=img.size[1]
                dic={}
                blue_coord=[]
                red_coord=[]
                yellow_coord=[]
                for y in range(height):
                    for x in range(width):
                        if img.getpixel((x,y)) == (0,0,255): #blue stuffs
                            blue_coord.append((x,y))
                        elif img.getpixel((x,y)) == (255,0,0): #red stuffs
                            red_coord.append((x,y))
                        elif img.getpixel((x,y)) == (255,255,0): #yellow stuffs
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
                self.nb_basic_seg=len(ref_segments)
            elif i==1:
                self.ref_noise=ref_segments
                self.nb_noise_seg=len(ref_segments)

      
        
    def write_ref_file(self):

        tests=[self.basic_test,self.noise_test]
        
        for j,test in enumerate(tests):
            for k in range(len(test)):
                img=test[k]
                width=img.size[0]
                height=img.size[1]
                if j==0:
                    path='DataSet/Reference_seg/basic'+str(k+1)+'.seg'
                    f=open(path,'w')
                    if k==0:
                        f.write('width\t'+str(width)+'\nheight\t'+str(height)+'\nsegments\t1\n')
                    else:
                        f.write('width\t'+str(width)+'\nheight\t'+str(height)+'\nsegments\t2\n')
                elif j==1:
                    path='DataSet/Reference_seg/noise'+str(k+1)+'.seg'
                    f=open(path,'w')
                    f.write('width\t'+str(width)+'\nheight\t'+str(height)+'\nsegments\t3\n')

                for y in range(height):
                    x=0
                    while x<width:
                        for i,color in enumerate([(0,0,0),(0,0,255),(255,0,0),(255,255,0),(0,0,0)]):
                            if img.getpixel((x,y))==color :
                                xmin=x
                                x_tmp=x
                                while img.getpixel((x_tmp,y))==color and x_tmp<width-1:
                                    x_tmp+=1
                                if i==4:
                                    i=0
                                f.write(str(i)+'\t'+str(y)+'\t'+str(xmin)+'\t'+str(x_tmp)+'\n')
                                if x_tmp==width-1:
                                    x=0
                                    break
                                else:
                                    x=x_tmp
                        break
                                
                f.close()
                  


def PIL2array(PIL_images):
    images=[]
    for i,img in enumerate(PIL_images):
        img=img.rotate(270)
        images.append(np.array(img.getdata(),np.uint8).reshape(img.size[1], img.size[0], 3))
    return images


imgs=ImagesTest()
imgs.get_pixels_coord()
#imgs.write_ref_file()

