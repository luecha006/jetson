"""evaluate.py

This script is used to evalute trained ImageNet models.
"""


import sys
import argparse

import tensorflow as tf

from config import config
from utils.utils import config_keras_backend, clear_keras_session
from utils.dataset import get_dataset
from models.adamw import AdamW


DESCRIPTION = """For example:
$ python3 evaluate.py --dataset_dir ${HOME}/data/ILSVRC2012/tfrecords \
                      --batch_size  64 \
                      saves/mobilenet_v2-model-final.h5
"""


def main():
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('--dataset_dir', type=str,
                        default=config.DEFAULT_DATASET_DIR)
    parser.add_argument('--batch_size', type=int, default=16)
    parser.add_argument('model_file', type=str,
                        help='a saved model (.h5) file')
    args = parser.parse_args()
    config_keras_backend()
    if not args.model_file.endswith('.h5'):
        sys.exit('model_file is not a .h5')
    model = tf.keras.models.load_model(
        args.model_file,
        compile=False,
        custom_objects={'AdamW': AdamW})
    model.compile(
        optimizer='sgd',
        loss='categorical_crossentropy',
        metrics=['accuracy'])
    ds_validation = get_dataset(
        args.dataset_dir, 'validation', args.batch_size)
    results = model.evaluate(
        x=ds_validation,
        steps=50000 // args.batch_size)
    print('test loss, test acc:', results)
    clear_keras_session()


if __name__ == '__main__':
    main()
