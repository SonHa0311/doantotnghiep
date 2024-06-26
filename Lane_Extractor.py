import cv2
import numpy as np
from Detection.Lanes.Morph_op import BwareaOpen,RetLargestContour,RetLargestContour_OuterLane,RetClosestContour2,Ret_LowestEdgePoints
import time
import config

HLS=0
src=0

Hue_Low = 0#74#83 63
Hue_High = 255
#Lit_Low = 95#174#112 
Lit_Low = 0 #107#174#112 
Sat_Low = 0

Hue_Low_Y = 0 #18
#Hue_High_Y = 49#42#38
Hue_High_Y = 42#42#38
Lit_Low_Y = 0
Sat_Low_Y = 10  #17  

use_edge_first = False

def OnHueLowChange(val):
    global Hue_Low
    Hue_Low = val
    MaskExtract()
def OnHueHighChange(val):
    global Hue_High
    Hue_High = val
    MaskExtract()
def OnLitLowChange(val):
    global Lit_Low
    Lit_Low = val
    MaskExtract()
def OnSatLowChange(val):
    global Sat_Low
    Sat_Low = val
    MaskExtract()

def OnHueLowChange_Y(val):
    global Hue_Low_Y
    Hue_Low_Y = val
    MaskExtract()
def OnHueHighChange_Y(val):
    global Hue_High_Y
    Hue_High_Y = val
    MaskExtract()
def OnLitLowChange_Y(val):
    global Lit_Low_Y
    Lit_Low_Y = val
    MaskExtract()
def OnSatLowChange_Y(val):
    global Sat_Low_Y
    Sat_Low_Y = val
    MaskExtract()

def MaskExtract():
    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(Hue_High       ,255,255))
    mask_Y = clr_segment(HLS,(Hue_Low_Y,Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,255,255))#Combine 6ms
    mask_Y_ = mask_Y != 0
    dst_Y = src * (mask_Y_[:,:,None].astype(src.dtype))
    mask_ = mask != 0

    dst = src * (mask_[:,:,None].astype(src.dtype))
    cv2.imshow("Loc_Mau_Trang",dst)
    cv2.imshow("Loc_Mau_Vang",dst_Y)

if(config.clr_segmentation_tuning):
    cv2.namedWindow("Loc_Mau_Trang",cv2.WINDOW_NORMAL)
    cv2.namedWindow("Loc_Mau_Vang",cv2.WINDOW_NORMAL)

    cv2.createTrackbar("Hue_L","Loc_Mau_Trang",Hue_Low,255,OnHueLowChange)
    cv2.createTrackbar("Hue_H","Loc_Mau_Trang",Hue_High,255,OnHueHighChange)
    cv2.createTrackbar("Lit_L","Loc_Mau_Trang",Lit_Low,255,OnLitLowChange)
    cv2.createTrackbar("Sat_L","Loc_Mau_Trang",Sat_Low,255,OnSatLowChange)

    cv2.createTrackbar("Hue_L","Loc_Mau_Vang",Hue_Low_Y,255,OnHueLowChange_Y)
    cv2.createTrackbar("Hue_H","Loc_Mau_Vang",Hue_High_Y,255,OnHueHighChange_Y)
    cv2.createTrackbar("Lit_L","Loc_Mau_Vang",Lit_Low_Y,255,OnLitLowChange_Y)
    cv2.createTrackbar("Sat_L","Loc_Mau_Vang",Sat_Low_Y,255,OnSatLowChange_Y)

def clr_segment(HSL,lower_range,upper_range):
    # 2. Performing Color Segmentation on Given Range
    lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
    upper = np.array( [upper_range[0]    ,255     ,255])
    mask = cv2.inRange(HSL, lower, upper)
    # 3. Dilating Segmented ROI's
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask

def LaneROI(frame,mask,minArea):
    if not use_edge_first:
        frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)# R & R = R
        Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY)
        #start_To = time.time()
        Lane_gray_opened = BwareaOpen(Lane_gray,minArea)#4 msec
        #end_To = time.time()
        #print("**** BWarea Open Loop took ",end_To - start_To," sec <-->  ")
        #Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray,mask=Lane_gray_opened)# R & R = R
        Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)# R & R = R
        Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1)
        Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3)#3msec
        #cv2.imshow('Lane_gray_Smoothed',Lane_gray_Smoothed)
        #cv2.imshow('MidLane_edge',Lane_edge)
    else:
        Lane_frame_edge = cv2.Canny(frame,50,150, None, 3)#3msec
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        Lane_frame_edge = cv2.morphologyEx(Lane_frame_edge, cv2.MORPH_CLOSE, kernel)
        #Lane_frame_edge[Lane_frame_edge.shape[0]-1:Lane_frame_edge.shape[0],0:int(Lane_frame_edge.shape[1]/8)] = 255
        #Lane_frame_edge[Lane_frame_edge.shape[0]-35:Lane_frame_edge.shape[0],0:1] = 255
        roi_masks = np.zeros_like(mask)
        cnts = cv2.findContours(Lane_frame_edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
        roi_masks = cv2.drawContours(roi_masks, cnts, -1, 255,-1)
        
        roi_masks = cv2.morphologyEx(roi_masks, cv2.MORPH_OPEN, kernel)

        frame_edges = cv2.bitwise_and(frame,frame,mask=roi_masks)# R & R = R

        HLS_edges = cv2.cvtColor(frame_edges,cv2.COLOR_BGR2HLS)#2 msc
        mask_clr_edges   = clr_segment(HLS_edges,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(Hue_High       ,255,255))
        Lane_edge = cv2.Canny(mask_clr_edges,50,150, None, 3)#3msec
        Lane_gray_opened = mask_clr_edges


        cv2.imshow('[Testing] MidLane_frame_edge',Lane_frame_edge)
        cv2.imshow('[Testing] MidLane_frame_mask',roi_masks)
        cv2.imshow('[Testing] MidLane_edges_clr',mask_clr_edges)

    #  Selecting Only ROI from Image
    # ROI_mask = np.zeros(Lane_edge.shape, dtype=np.uint8)
    # cv2.rectangle(ROI_mask,(0,CropHeight_resized),(Lane_edge.shape[1],Lane_edge.shape[0]),255,thickness=-1)
    # Lane_edge_ROI = cv2.bitwise_and(Lane_edge,Lane_edge,mask=ROI_mask)
    # Lane_ROI_mask = cv2.bitwise_and(Lane_gray_opened,Lane_gray_opened,mask=ROI_mask)

    #cv2.imshow('ROI_mask',ROI_mask)
    #cv2.imshow('frame_Lane',frame_Lane)
    #cv2.imshow('Lane_gray',Lane_gray)
    #cv2.imshow('Lane_gray_opened',Lane_gray_opened)
    #cv2.imshow('Lane_edge_ROI',Lane_edge_ROI)

    #return Lane_edge_ROI,Lane_ROI_mask
    return Lane_edge,Lane_gray_opened

def ROI_extracter(image,mask,strtPnt):
    #  Selecting Only ROI from Image
    ROI_mask = np.zeros(image.shape, dtype=np.uint8)
    cv2.rectangle(ROI_mask,strtPnt,(image.shape[1],image.shape[0]),255,thickness=-1)
    #cv2.imshow('ROI_mask',ROI_mask)
    image_ROI = cv2.bitwise_and(image,image,mask=ROI_mask)
    image_ROI_mask = cv2.bitwise_and(mask,mask,mask=ROI_mask)
    return image_ROI,image_ROI_mask

def OuterLaneROI(frame,mask,minArea):
    Outer_Points_list=[]
    frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)# R & R = R
    Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY)
    Lane_gray_opened = BwareaOpen(Lane_gray,minArea)
    #Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray,mask=Lane_gray_opened)# R & R = R
    Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)# R & R = R
    Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1)
    Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3)
    #  Selecting Only ROI from Image
    #Lane_edge_ROI,Lane_ROI_mask = ROI_extracter(Lane_edge,Lane_gray_opened,(0,CropHeight_resized))
    #ROI_mask_Largest,Largest_found = RetLargestContour_OuterLane(Lane_ROI_mask,minArea)
    ROI_mask_Largest,Largest_found = RetLargestContour_OuterLane(Lane_gray_opened,minArea)
    if(Largest_found):
        Outer_edge_lane_final = cv2.bitwise_and(Lane_edge,ROI_mask_Largest)
        #cv2.namedWindow("ROI_mask_Largest",cv2.WINDOW_NORMAL)
        #cv2.imshow('ROI_mask_Largest',ROI_mask_Largest)
        #Lane_OneSide ,Lane_OneSide_found,Outer_Points_list = RetClosestContour2(ROI_mask_Largest)
        Lane_TwoEdges, Outer_Points_list = Ret_LowestEdgePoints(ROI_mask_Largest)
        Lane_edge = Outer_edge_lane_final
        #Lane_edge_ROI = Outer_edge_lane_final
    else:
        Lane_TwoEdges=np.zeros(Lane_gray.shape,Lane_gray.dtype)


    #return Lane_edge,Lane_gray_opened,Lane_OneSide,Outer_Points_list
    return Lane_edge,Lane_TwoEdges,Outer_Points_list

def GetLaneROI(frame,minArea):

    global HLS,src
    src = frame.copy()
    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc
    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(Hue_High       ,255,255))
    mask_Y = clr_segment(HLS,(Hue_Low_Y,Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,255,255))#Combine 6ms
    #Outer_edge_ROI,_,OuterLane_OneSide,Outer_Points_list = OuterLaneROI(frame,mask_Y,minArea+500)#27msec

    Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list = OuterLaneROI(frame,mask_Y,minArea+500)#27msec
    Mid_edge_ROI,Mid_ROI_mask = LaneROI(frame,mask,minArea)#20 msec

    if(config.clr_segmentation_tuning):
        cv2.imshow("Loc_Mau_Trang",mask)
        cv2.imshow("Loc_Mau_Vang",mask_Y)
        
    return Mid_edge_ROI,Mid_ROI_mask,Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list