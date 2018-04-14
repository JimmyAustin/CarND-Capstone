from skimage import io
from skimage import transform as tf
import random
from scipy import ndimage
import scipy
import numpy as np

def permutate(image, label, color_blur=True, color_blur_amount=1, 
                            blur_or_sharpen=True, sharpen_proportion=0.5, max_blur_amount=5, max_sharpen=30, 
                            skew=True, random_skew_range=0.2,
                            scale=True, max_zoom_amount=0.2, shift_amount=0.1):
    if scale is True:
        scale_down_amount = 1 - random.uniform(0.0, max_zoom_amount)
        slide_amount = random.uniform(0.0, shift_amount)

        new_width = int(scale_down_amount * image.shape[0])
        new_height = int(scale_down_amount * image.shape[1])
        scale_width = int(slide_amount * image.shape[0])
        scale_height = int(slide_amount * image.shape[1])

        resized_image=image[scale_width:scale_width + new_width,scale_height:scale_height + new_height]
        resized_label=label[scale_width:scale_width + new_width,scale_height:scale_height + new_height]
    
        image = scipy.misc.imresize(resized_image, image.shape)

        reshaped_label = np.resize(resized_label, 
                                   (resized_label.shape[0], resized_label.shape[1]))
        reshaped_label = scipy.misc.imresize(reshaped_label, label.shape)
        label = np.resize(reshaped_label, 
                          (reshaped_label.shape[0], reshaped_label.shape[1], 1))

    if blur_or_sharpen is True:
        if random.random() > sharpen_proportion:
            #Sharpen
            filter_blurred_f = ndimage.gaussian_filter(image, 1)
            #image = image + max_sharpen * (image - filter_blurred_f)
        else:
            #Blur
            blur_amount = max_blur_amount*random.random()
            image = ndimage.gaussian_filter(image, (blur_amount, blur_amount, 0))
    
    if color_blur is True:
        #Blur
        blur_amount = max_blur_amount*random.random()
        image = ndimage.gaussian_filter(image, (0, 0, color_blur_amount))
    if skew is True:
        shear_amount = random.uniform(-random_skew_range, random_skew_range)
        afine_tf = tf.AffineTransform(shear=shear_amount)

        # Apply transform to image data
        image = tf.warp(image, inverse_map=afine_tf)
        label = tf.warp(label, inverse_map=afine_tf)
    
    if image.max() > 1.0:
        image = (image * 255.0).astype(np.uint8)
        label = label.astype(np.uint8)
        
    return image, label
