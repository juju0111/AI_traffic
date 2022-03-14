# -*- coding: utf-8 -*-
import tensorflow as tf
from tensorflow.keras.layers import Input, Dense
from tensorflow.keras.optimizers import Adamax
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

input_size = 16
output_size = 4
node_num = 200
learning_rate = 0.001
epochs = 500
batch_size = 16

df = pd.read_excel('data_set/data_set.xlsx')

df = df.to_numpy()

x = np.array(
    [df[3:, 1], df[3:, 3], df[3:, 4], df[3:, 6], df[3:, 7], df[3:, 9], df[3:, 10], df[3:, 12], df[3:, 13], df[3:, 15],
     df[3:, 16], df[3:, 18], df[3:, 19], df[3:, 21], df[3:, 22], df[3:, 24], df[3:, 33], df[3:, 34], df[3:, 35],
     df[3:, 36]])
x = x.T

a = x[:, :16]
b = x[:, 16:]


def mish(x):
    return x * tf.math.tanh(tf.math.softplus(x))


x_train, x_test, y_train, y_test = train_test_split(a, b, test_size=0.3, shuffle=True, random_state=34)  

x_test, x_valid, y_test, y_valid = train_test_split(x_test, y_test, test_size=0.5, shuffle=True,
                                                    random_state=14)

model = tf.keras.Sequential(
    [
        Input(shape=input_size, name='Input states'),
        Dense(node_num, activation='relu', name='layer1', kernel_initializer='he_normal'),
        Dense(node_num, activation='relu', name='layer2', kernel_initializer='he_normal'),
        Dense(node_num, activation='relu', name='layer3', kernel_initializer='he_normal'),
        Dense(node_num, activation='relu', name='layer4', kernel_initializer='he_normal'),
        Dense(node_num, activation='relu', name='layer5', kernel_initializer='he_normal'),
        Dense(output_size, activation='linear', name='Outputs')
    ]
)

model.compile(optimizer=Adamax(learning_rate=learning_rate), loss='mean_squared_error',
              metrics=['mae'])
model.summary()
checkpoint = ModelCheckpoint('mode/new_model.h5', monitor='val_mae', verbose=0,
                             save_best_only=True, mode='auto')

# es = EarlyStopping(monitor='val_loss', patience=10)

hist = model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, validation_data=(x_valid, y_valid),
                 callbacks=[checkpoint])

fig, loss_ax = plt.subplots()

acc_ax = loss_ax.twinx()

loss_ax.plot(hist.history['loss'], 'r', label='train loss')
loss_ax.plot(hist.history['val_loss'], 'g', label='val loss')

acc_ax.plot(hist.history['mae'], 'b', label='train error')
acc_ax.plot(hist.history['val_mae'], 'y', label='val error')

loss_ax.set_xlabel('epoch')
loss_ax.set_ylabel('loss')
acc_ax.set_ylabel('MAE')

loss_ax.legend(loc='upper left')
acc_ax.legend(loc='lower left')

plt.show()

plt.savefig('plot_mse.png')

score = model.evaluate(x_test, y_test, verbose=0)
print('?Öå?ä§?ä∏ ?Üê?ã§Í∞? ', score[0])
print('?Öå?ä§?ä∏ MAE ', score[1])
