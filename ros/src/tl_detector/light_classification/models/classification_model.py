from keras.models import Sequential
from keras.layers import Conv2D, Flatten, Dense, MaxPooling2D, Dropout
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers

def get_model(dropout_perc=0.0, weights_file=None):
    num_classes = 4
    model = Sequential()
    model.add(Conv2D(32, (3, 3), input_shape=(64, 32, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
    model.add(MaxPooling2D(2,2))
    Dropout(dropout_perc)
    model.add(Conv2D(32, (3, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
    model.add(MaxPooling2D(2,2))
    Dropout(dropout_perc)
    model.add(Flatten())

    #model.add(Dense(128, activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
    model.add(Dense(8, activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
    model.add(Dense(num_classes, activation='softmax'))
    loss = losses.categorical_crossentropy
    optimizer = optimizers.Adam()
    model.compile(loss=loss, optimizer=optimizer, metrics=['accuracy'])

    if weights_file:
        model.load_weights(weights_file)

    return model