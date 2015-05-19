import segmentation

def read_seg_file(name,berkeley=False):

    #Open the seg file
    if berkeley:
        files=open_seg_files(name)
    else:
        files=[]
        path='DataSet/Reference_seg/'+name+'.seg'
        f=open(path,'r')
        files.append(f)

    ref=segmentation.SrObjectSegmentation(name,{},0)
  
    for f in files:      
        #Header
        if not berkeley:
            for i in range(3):
                f.readline()
        else:
            for i in range(11):
                f.readline()

        
        #Reading file
        lines=[]
        for line in f:
            lines.append([int(x) for x in line.split()])
        f.close()

        #Dictionnary with segments id as keys and coordinates as values
        dic={}
        coordinates=[]
        for line in lines:
            for y in range(line[2],line[3]):
                coordinates.append((line[1],y))
            dic[line[0]]=coordinates


        #Sort by descending size of segments

        seg_by_length=sorted(dic.values(),key=len,reverse=True) #delete the background (biggest segment)
        dic_fin={}
        for i in range(len(seg_by_length)):
            dic_fin[i]=seg_by_length[i]

        ref.points=dic_fin
        ref.nb_segments=len(dic_fin.values())

    return ref 



def open_seg_files(name):
        
    #Open the seg files
    files=[]
    i=1
    while True:
        path='DataSet/Reference_seg/'+name+'_'+str(i)+'.seg'
        try:
            f=open(path,'r')
        except IOError:
            break
        else:
            files.append(f)
            i+=1
    return files

#read_seg_file('lvl1_1')
#read_seg_file('3096',berkeley=True)
