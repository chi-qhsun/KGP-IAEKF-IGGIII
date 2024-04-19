from sklearn.datasets import make_regression
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from pyswarms.single.global_best import GlobalBestPSO
import numpy as np

class KGP:
    def __init__(self):
        self.params = {
    'gbdt_params': {
        'n_estimators': 100,
        'learning_rate': 0.5,
        'max_depth': 8,
        'min_samples_split': 2,
        'min_samples_leaf': 2,
    },
    'pso_params': {
        'n_particles': 50,  #Hyper
        'dimensions': 5,    #Equal to Param Number of GBDT
        'options': {
            'c1': 0.103,    #Hyper
            'c2': 2.897,    #Hyper
            'w': 0.6,       #Hyper
        },
        'iters': 100        #Hyper
    }
}
        self.GBDT_model = GradientBoostingRegressor(**self.params['gbdt_params'])
        self.pso = GlobalBestPSO(n_particles=self.params['pso_params']['n_particles'],
                                 dimensions=self.params['pso_params']['dimensions'],
                                 options=self.params['pso_params']['options'])
        self.X = []
        self.Y = []
        self.X_train = []
        self.X_val = []
        self.y_train = []
        self.y_val = []

    def Add_x_y(self,X,Y):
        if (len(self.X) <= 1000):
            self.X.append(X)
            self.Y.append(Y)
        else:
            print("Maximum capacity of 1000 data points reached.")
    
    def train_test_split(self):
        self.X_train, self.X_val, self.y_train, self.y_val = train_test_split(self.X, self.Y, test_size=0.3, random_state=42)
        
    def fitness_function(self, particle):
        # Particle position represents GBDT hyperparameters
        n_estimators = int(particle[0])
        learning_rate = particle[1]  # Ensure learning rate is between 0.01 and 1 for stability
        max_depth = int(particle[2])
        min_samples_split = int(particle[3])
        min_samples_leaf = int(particle[4])

        # Construct the GBDT model with particle's hyperparameters
        gbdt = GradientBoostingRegressor(
            n_estimators=n_estimators,
            learning_rate=max(min(learning_rate, 1), 0.01),  # Clamp to [0.01, 1] range
            max_depth=max_depth,
            min_samples_split=min_samples_split,
            min_samples_leaf=min_samples_leaf
        )
        
        # Train the model
        gbdt.fit(self.X_train, self.y_train)
        
        # Calculate MSE on the validation set
        predictions = gbdt.predict(self.X_val)
        mse = mean_squared_error(self.y_val, predictions)
        
        return mse
        
        
    def optimize_gbdt_with_pso(self):
        # Use PSO to optimize GBDT hyperparameters
        best_cost, best_pos = self.pso.optimize(self.fitness, iters=self.params['pso_params']['iters'],verbose = True)
        return best_cost, best_pos
        
    def update_gbdt_params(self, best_pos):
        # Update GBDT parameters with the best found by PSO
        best_params = {
        'n_estimators': int(best_pos[0]),
        'learning_rate': best_pos[1],
        'max_depth': int(best_pos[2]),
        'min_samples_split': int(best_pos[3]),
        'min_samples_leaf': int(best_pos[4])
        }
        self.GBDT_model.set_params(**best_params)
