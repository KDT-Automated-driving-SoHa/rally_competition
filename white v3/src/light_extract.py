import cv2
import numpy as np
 
def change(img):
    #img = cv2.imread('c1.PNG')
    img_YUV = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)    
    y = img_YUV[:,:,0]

    rows = y.shape[0]    
    cols = y.shape[1]
 

    imgLog = np.log1p(np.array(y, dtype='float') / 255) 
 

    M = 2*rows + 1
    N = 2*cols + 1
 

    sigma = 10
    (X, Y) = np.meshgrid(np.linspace(0, N-1, N), np.linspace(0, M-1, M)) 
    Xc = np.ceil(N/2) 
    Yc = np.ceil(M/2)
    gaussianNumerator = (X - Xc)**2 + (Y - Yc)**2 


    LPF = np.exp(-gaussianNumerator / (2*sigma*sigma))
    HPF = 1 - LPF
 

    LPF_shift = np.fft.ifftshift(LPF.copy())
    HPF_shift = np.fft.ifftshift(HPF.copy())



    img_FFT = np.fft.fft2(imgLog.copy(), (M, N))
    img_LF = np.real(np.fft.ifft2(img_FFT.copy() * LPF_shift, (M, N)))
    img_HF = np.real(np.fft.ifft2(img_FFT.copy() * HPF_shift, (M, N)))


    gamma1 = 0.3
    gamma2 = 1.5 #1.3
    img_adjusting = gamma1*img_LF[0:rows, 0:cols] + gamma2*img_HF[0:rows, 0:cols]

    img_exp = np.expm1(img_adjusting) 
    img_exp = (img_exp - np.min(img_exp)) / (np.max(img_exp) - np.min(img_exp)) 
    img_out = np.array(255*img_exp, dtype = 'uint8') 
 

    img_YUV[:,:,0] = img_out
    result = cv2.cvtColor(img_YUV, cv2.COLOR_YUV2BGR)

    return result
#cv2.imshow('homomorphic', result)
#cv2.imshow("1", img)
#cv2.waitKey(0)
