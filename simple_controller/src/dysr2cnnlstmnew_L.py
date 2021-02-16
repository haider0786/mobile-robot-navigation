import keras
import numpy as np
from keras.layers import Conv2D, MaxPooling2D, Conv1D, MaxPooling1D, GlobalAveragePooling1D, Flatten
from keras.layers import Input, Embedding, LSTM, Dense
from keras.layers import Input, Embedding, LSTM, Dense
from keras.models import Model
from numpy import inf
import pandas as pd
import glob,os
from math import sqrt
from numpy import concatenate
from matplotlib import pyplot
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Dropout
import pydot_ng
import pydot
import math
import csv
from keras.utils import plot_model

time_step = 4


'''Reading of csv file and making a single array'''
li = []
path = r'/home/haider/data_collected/dy_datasr2/dy_datasr2_10hz/laser_plus_position/linear'
def read(path):


    all_files = glob.glob(os.path.join(path, "dysr2_las*.csv"))

    for filename in all_files:

        df = pd.read_csv(filename, index_col=0, header=0)
        li.append(df)

    frame = pd.concat(li, axis=0, ignore_index=True)
    return frame
#frame = read(path)
#print('frame_shape=', frame.shape)

max_value = 18
min_value = -5

# convert series to supervised learning
def series_to_supervised(data, n_in=1, n_out=1, dropnan=True):
    n_vars = 1 if type(data) is list else data.shape[1]
    df = DataFrame(data)
    cols, names = list(), list()
    # input sequence (t-n, ... t-1)
    for i in range(n_in, 0, -1):
        cols.append(df.shift(i))
        names += [('var%d(t-%d)' % (j + 1, i)) for j in range(n_vars)]
    # forecast sequence (t, t+1, ... t+n)
    for i in range(0, n_out):
        cols.append(df.shift(-i))
        if i == 0:
            names += [('var%d(t)' % (j + 1)) for j in range(n_vars)]
        else:
            names += [('var%d(t+%d)' % (j + 1, i)) for j in range(n_vars)]
    # put it all together
    agg = concat(cols, axis=1)
    agg.columns = names
    # drop rows with NaN values
    if dropnan:
        agg.dropna(inplace=True)
    return agg

def data_conv(data, length, Np, actual_len):
    sumd = 0
    counter = 0
    j = 0
    a_data = np.zeros(length)
    for i in range(0, actual_len, 1):
        counter = counter + 1
        sumd = sumd + data[i]
        if counter == Np:
            a_data[j] = sumd/Np
            counter = 0
            sumd = 0
            j = j+1
    return a_data

# load dataset

dataset = read(path)
values = dataset.values
print(values)
print(values.shape)
Train_size = int(0.9 * values.shape[0])

print Train_size
values[values == inf] = 20  ## Replace the infinity with the maximum limit range of laser , maxumum range = 30 meter
values = values.astype('float32')
reframed = series_to_supervised(values, 1, 1)
k = list(range((int(values.shape[1])-1), (int(values.shape[1])*2-1)))
print('k=', k)
print('len_k=', len(k))
# drop columns we don't want to predictdtype='int32'
reframed.drop(reframed.columns[k], axis=1, inplace=True)
print(reframed.head())


values = reframed.values
value1 = pd.DataFrame(values)
#value1.to_csv('value1.csv')
scaler = MinMaxScaler(feature_range=(-1, 1))
scaled = scaler.fit_transform(values)

print("shape of scaled=", scaled.shape)
print('scaled_shape[0]', scaled.shape[0])
print('scaled_shape[1]', scaled.shape[1])

Train1 = scaled[:Train_size, :]
Test1 = scaled[Train_size:scaled.shape[0], :]

div_train = Train1.shape[0] % time_step
div_test = Test1.shape[0] % time_step
if div_test != 0:
    Test = Test1[:-div_test, :]
else:
    Test = Test1
if div_train != 0:
    Train = Train1[:-div_train, :]
else:
    Train = Train1

print 'div_train', div_train
print 'div_test', div_test

len_Train = Train.shape[0]/time_step
len_Test = Test.shape[0]/time_step
print 'len_Train', len_Train
print 'len_Test', len_Test
O_Train = data_conv(Train[:, -1], len_Train, time_step, Train.shape[0])
O_Test = data_conv(Test[:, -1], len_Test, time_step, Test.shape[0])
print 'O_Train', O_Train.shape
print 'O_Test', O_Test.shape




Input1_Train = Train[:, :-3]
Input2_Train = Train[:, -3:-1]
Output_Train = O_Train

print('Input1_Train_shape', Input1_Train.shape)
print('Input2_Train_shape', Input2_Train.shape)
print('Output_Train_shape', Output_Train.shape)

Input1_Test = Test[:, :-3]
Input2_Test = Test[:, -3:-1]
Output_Test = O_Test
print('Input1_Test_shape', Input1_Test.shape)
print('Input2_Test_shape', Input2_Test.shape)
print('Output_Test_shape', Output_Test.shape)
O_Test = DataFrame(O_Test)
O_Train= DataFrame(O_Train)
O_Test.to_csv('O_Test.csv')
O_Train.to_csv('O_Train.csv')




print 'Input1_Train', Input1_Train.shape
print 'Input1_Test', Input1_Test.shape

print 'Input2_Train', Input2_Train.shape
print 'Input2_Test', Input2_Test.shape
#print Output_Test
print Output_Train

#The first input

#main_input = Input(shape=(time_step, Input1_Train.shape[1]), dtype='float32',  name='main_input')
main_input1 = Input(shape=(time_step, Input1_Train.shape[1]), dtype='float32',  name='main_input1')
cnn_out1 = Conv1D(512, kernel_size=3, activation='relu')(main_input1)
lstm_out1 = LSTM(512, return_sequences=True, activation='relu')(cnn_out1)
lstm_out2 = Dropout(0.5)(lstm_out1)
added1 = keras.layers.Add()([lstm_out2, cnn_out1])
lstm_out3 = LSTM(150, activation='relu')(added1)
lstm_out4 = Dropout(0.5)(lstm_out3)
lstm_out5 = Dense(150, activation='relu')(lstm_out4)
added2 = keras.layers.Add()([lstm_out5, lstm_out3])
# This embedding layer will encode the input sequence
# into a sequence of dense 512-dimensional vectors.
#x = Embedding(output_dim=1000, input_dim=10000, input_length=1020)(main_input)



# A LSTM will transform the vector sequence into a single vector,
# containing information about the entire sequence

# cnn_out = Conv1D(512, kernel_size=3, activation='relu')(main_input)
# lstm_out = LSTM(250, return_sequences=True, activation='relu')(cnn_out)
# lstm_out = Dropout(0.5)(lstm_out)
# lstm_out = LSTM(150,  activation='relu')(lstm_out)
# lstm_out = Dropout(0.5)(lstm_out)
# lstm_out = Dense(100, activation='relu')(lstm_out)


#print('lstm_out', lstm_out.shape)

#auxiliary_output = Dense(1, activation='sigmoid', dtype='float32', name='aux_output')(lstm_out)

# Second input

auxiliary_input = Input(shape=(time_step, Input2_Train.shape[1]), dtype='float32', name='aux_input')
#auxiliary_input1 = Input(shape=(5, Input2_Train.shape[1]), dtype='float32', name='aux_input1')

#x = keras.layers.concatenate([lstm_out1, lstm_out, auxiliary_input, auxiliary_input1])

# We stack a deep densely-connected network on top
aux_output = LSTM(50, activation='relu')(auxiliary_input)
x = keras.layers.concatenate([aux_output, added2])

x = Dense(32, activation='relu')(x)

# And finally we add the main logistic regression layer

main_output = Dense(1, activation='linear', name='main_output')(x)

model = Model(inputs=[main_input1,  auxiliary_input], outputs=[main_output])

#model.compile(optimizer='rmsprop', loss='binary_crossentropy', loss_weights=[1])

model.compile(loss='mae', optimizer='adam', metrics=['accuracy'], loss_weights=None)
model.summary()
#Output_Train = Output_Train.reshape(Train_size, 1)

Input1_Train = Input1_Train.reshape((Output_Train.shape[0], time_step, Input1_Train.shape[1]))
Input1_Test = Input1_Test.reshape((Output_Test.shape[0], time_step, Input1_Test.shape[1]))
Input2_Train = Input2_Train.reshape((Output_Train.shape[0], time_step, Input2_Train.shape[1]))
Input2_Test = Input2_Test.reshape((Output_Test.shape[0], time_step, Input2_Test.shape[1]))
#print(Output_Train.shape)

#print(Output_Train[-1])

#Input1_Train = Input1_Train.reshape((Input1_Train.shape[0], 1, Input1_Train.shape[1]))
#Input2_Train = Input2_Train.reshape((Input2_Train.shape[0], 1, 2))
#Input2_Train = Input2_Train.reshape((1000, 1, 2))

#Input1_Test = Input1_Test.reshape((Input1_Test.shape[0], 1, Input1_Test.shape[1]))
#Input2_Test = Input2_Test.reshape((Input2_Test.shape[0], 1, 2))
#Input2_Test = Input2_Test.reshape((580, 1, 2))

#model.fit([Input1_Train, additional_data], [Output_Train], epochs=1, batch_size=100)

headline_data = Input1_Train
history = model.fit([Input1_Train, Input2_Train], [Output_Train], validation_data=([Input1_Test, Input2_Test], Output_Test),
                    epochs=1000, batch_size=72, verbose=2, shuffle=True)
#Input1_Test = Input1_Test.reshape((Input1_Test.shape[0], 1, 1020))

#Input2_Test = Input2_Test.reshape((580, 1, 2))print('scaled_shape[0]', scaled.shape[0])
plot_model(model, to_file='multiple_outputs.png')
model.save('dy_sr2_4step_10hz_L_loopmodel.h5')

y = model.predict([Input1_Test, Input2_Test])
print(y)

# calculate RMSE
rmse = sqrt(mean_squared_error(y, Output_Test))
print('Test RMSE: %.3f' % rmse)

pyplot.plot(history.history['loss'], label='train')
pyplot.plot(history.history['val_loss'], label='test')
pyplot.legend()
pyplot.show()

pyplot.plot(Output_Test, color='green', label='Y_test')
pyplot.plot(y, color='red', label='Predict')
pyplot.legend()
pyplot.show()

pyplot.plot(history.history['val_accuracy'], color='green', label='Val_acc')
pyplot.plot(history.history['accuracy'], color='red', label='Accuracy')
pyplot.legend()
pyplot.show()