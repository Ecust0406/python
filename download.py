#!coding:utf-8
import os
#from multiprocessing.pool import Pool
#pool = Pool(processes=1)
def download(item, test):
    print test
    os.system("aws s3 cp s3://momenta-tuling-share-data-result/%s %s --p result"%(item, "./" + item.split('/')[-2]))
    
with open("./bag_txt.txt", "r") as ft:
    items = ft.readline().split(" ")
    i = 0
    for item in items :
        i+=1
        item = item.strip()
        print item
        dst_dir = "./" + item.split('/')[-2]
        if not os.path.exists(dst_dir):
            os.makedirs(dst_dir)
        print("aws s3 cp s3://momenta-tuling-share-data-result/%s %s --p result"%(item, dst_dir))
        download(item, i)
