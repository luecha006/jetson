import argparse
# from cProfile import label
# from operator import mod

import numpy as np
import os, sys
import cv2
from threading import Thread


# from readcamera import opencamera
from display_sound import playsound

def preprocess(img):
    """Preprocess an image for Keras ImageNet model inferencing."""
    if img.ndim != 3:
        raise TypeError('bad ndim of img')
    if img.dtype != np.uint8:
        raise TypeError('bad dtype of img')
    img = cv2.resize(img, (224,224))
    rimg = np.array(img).reshape(224,224,3).astype(np.float32) / 255
    # rimg = rimg.astype('float32') / 255
    # rimg = np.reshape(rimg, (1,224,224,3))
    # cv2.imshow('img',img)
    # cv2.waitKey(0)
    return rimg


def infer_with_tf(img, model):
    """Inference the image with TensorFlow model."""
    import os
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    import tensorflow as tf
    from utils.utils import config_keras_backend, clear_keras_session
    from models.adamw import AdamW

    config_keras_backend()

    # load the trained model
    net = tf.keras.models.load_model(model, compile=False,
                                     custom_objects={'AdamW': AdamW})
    predictions = net.predict(img)
    clear_keras_session()

    return predictions


def init_trt_buffers(cuda, trt, engine):
    """Initialize host buffers and cuda buffers for the engine."""
    assert engine[0] == 'input_3:0'
    assert engine.get_binding_shape(0)[1:] == (224, 224, 3)
    size = trt.volume((1, 224, 224, 3)) * engine.max_batch_size
    host_input = cuda.pagelocked_empty(size, np.float32)
    cuda_input = cuda.mem_alloc(host_input.nbytes)
    assert engine[1] == 'dense_1/Softmax:0'
    assert engine.get_binding_shape(1)[1:] == (3,)
    size = trt.volume((1, 3)) * engine.max_batch_size
    host_output = cuda.pagelocked_empty(size, np.float32)
    cuda_output = cuda.mem_alloc(host_output.nbytes)
    return host_input, cuda_input, host_output, cuda_output


def infer_with_trt(img, model):
    print('model', model)
    """Inference the image with TensorRT engine."""
    import pycuda.autoinit
    import pycuda.driver as cuda
    import tensorrt as trt

    TRT_LOGGER = trt.Logger(trt.Logger.INFO)
    with open(model, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    assert len(engine) == 2, 'ERROR: bad number of bindings'
    host_input, cuda_input, host_output, cuda_output = init_trt_buffers(
        cuda, trt, engine)
    
    stream = cuda.Stream()
    context = engine.create_execution_context()
    
    context.set_binding_shape(0, (1, 224, 224, 3))
    np.copyto(host_input, img.ravel())
    cuda.memcpy_htod_async(cuda_input, host_input, stream)
    if trt.__version__[0] >= '7':
        context.execute_async_v2(bindings=[int(cuda_input), int(cuda_output)],
                                 stream_handle=stream.handle)
    else:
        context.execute_async(bindings=[int(cuda_input), int(cuda_output)],
                              stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(host_output, cuda_output, stream)
    stream.synchronize()
    return host_output


def main(img):
    print('main function')
    model = 'tensorrt/mobilev2_facemask_model.engine'

    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB )
    img = cv2.resize(img, (224, 224))
    img = preprocess(img)

    # predict the image
    if model.endswith('.h5'):
        predictions = infer_with_tf(img, model)
    elif model.endswith('.engine'):
        predictions = infer_with_trt(img, model)
    else:
        raise SystemExit('ERROR: bad model')

    # postprocess
    class_label = ['w', 'm', 'o']
    result = class_label[np.argmax(predictions)]
    
    # debug
    print('predictions is ',predictions)
    # print('result ', result)
    return result