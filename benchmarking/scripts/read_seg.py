

#Start with 1 user segmentation

def read_seg_file(id):
        
        #Open the seg file
        path='DataSet/Reference_seg/'+str(id)+'.seg'
        f=open(path,'r')
        
        #Header
        header_lg=11 
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
        seg_by_length=sorted(dic.values(),key=len,reverse=True)
        for i in range(len(dic)):
                dic[i]=seg_by_length[i]

        return dic


       
