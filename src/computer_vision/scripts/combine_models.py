# import the neccessary packages
from tensorflow.keras.models import load_model
from tensorflow.python.keras.models import clone_model
import matplotlib.pyplot as plt 

# python glob package is a module that finds all the pathnames matching a 
# specified pattern according to the rules used in the Unix shell, the results are returned in arbitary order.
import glob
import os 
import numpy as np 


"""This class will combine and average a list of models
Inspired by Jason Brownlee
"""

class CombineModels:

    def __init__(self,model_path,search_pattern_extension):
        # this will produce something like this path_name/*.models
        modelPaths = os.path.sep.join([model_path,search_pattern_extension]) 
        self.modelPaths = sorted(list(glob.glob(modelPaths)))
        print(self.modelPaths)
        self.models = []

        # load the models 
        self.load_models()
        if len(self.models) ==0:
            raise ValueError("The search pattern did not select any model files. Try again.")

    # load all the models using the directory and pathname seach pattern
    def load_models(self):
        self.models=[]
        for (i,modelPath) in enumerate(self.modelPaths):
            print("[INFO] loading model {}/{}".format(i+1,len(self.modelPaths)))
            self.models.append(load_model(modelPath))

    def load_models_n(self,n_members):
        self.models=[]
        for (i,modelPath) in enumerate(self.modelPaths[:n_members]):
            print("[INFO] loading model {}/{}".format(i+1,n_members))
            self.models.append(load_model(modelPath))

    # average the weights 
    def model_weight_ensemble(self,weights,plot_layer_weights=False):
        # determine how many layers need to be averaged
        n_layers = len(self.models[0].get_weights())

        if (plot_layer_weights):
            plt.style.use("ggplot")
            plt.figure()

        # create an average model weights
        avg_model_weights = list()

        for layer in range(n_layers):
            # collect this layer from each model
            
            if(plot_layer_weights):
                xrng= np.arange(0,self.models[0].get_weights()[layer].flatten().shape[0])
                for (cur,curr_model) in enumerate(self.models):
                    plt.plot(xrng, curr_model.get_weights()[layer].flatten(), label=str(cur))

            layer_weights = np.array([model.get_weights()[layer] for model in self.models])
            # weighted average of weights for this layer
            avg_layer_weights=np.average(layer_weights,axis=0,weights=weights)
            # append the weights to the list
            avg_model_weights.append(avg_layer_weights)
            
            if(plot_layer_weights):
                plt.plot(xrng,avg_layer_weights.flatten(),label='avg')
                plt.legend()
                plt.title("Layer {} Flattened Weights".format(layer))
                plt.xlabel("Vector Index")
                plt.ylabel("Vector Value")
                plt.show()
               
                plt.clf()

        avg_model_weights = np.asarray(avg_model_weights)

        print(self.models[0].get_weights()[0].shape,avg_model_weights[0].shape)
        print(np.linalg.norm(self.models[0].get_weights()[0].flatten()-avg_model_weights[0].flatten()))
       
        # create a new model with the same structure
        # cloning isn't working but when it does I'll add that here
        self.models[0].set_weights(avg_model_weights)
        
        return self.models[0]

    def evaluate_n_members(self,n_members):
        self.load_models_n(n_members)
        num_models = len(self.models)
        weights = [(1)/float(num_models) for i in range(0,num_models)]
        return self.model_weight_ensemble(weights)
