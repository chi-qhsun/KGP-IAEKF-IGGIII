from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor
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
        'iters': 10        #Hyper
    }
}
        self.GBDT_model = MultiOutputRegressor(GradientBoostingRegressor(**self.params['gbdt_params']))
        self.pso = GlobalBestPSO(n_particles=self.params['pso_params']['n_particles'],
                                 dimensions=self.params['pso_params']['dimensions'],
                                 options=self.params['pso_params']['options'],
                                 bounds=(np.array([50, 0.01, 4, 2, 1]), np.array([500, 1, 10, 20, 20])))
                                
        self.X = []
        self.Y = []
        self.X_train = []
        self.X_val = []
        self.y_train = []
        self.y_val = []
        self.triggered = False
        self.Model_prediction_count = 0
        self.Sampling_count = 0

    def Add_x_y(self,X,Y):
        if len(self.X) < 1000:
            if (self.Sampling_count) == 100:
                self.X.append(X)
                self.Y.append(Y)
                self.triggered = False
                self.Sampling_count = 0
            else:
                self.Sampling_count += 1
        else:
            self.Sampling_count += 1
            return True  # limit reached
        return False  # limit not reached

    def train_test_split(self):
        self.X = np.array(self.X)
        self.Y = np.array(self.Y)
        self.X_train, self.X_val, self.y_train, self.y_val = train_test_split(self.X, self.Y, test_size=0.3, random_state=42)
        
    def fitness_function(self, particles):
        mse_scores = np.zeros(particles.shape[0])
        for idx, particle in enumerate(particles):
            n_estimators = int(particle[0])
            learning_rate = max(min(particle[1], 1), 0.01)
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
            multioutput_gbdt = MultiOutputRegressor(gbdt)
            
            # Train the model
            multioutput_gbdt.fit(self.X_train, self.y_train)
            
            # Calculate MSE on the validation set
            predictions = multioutput_gbdt.predict(self.X_val)
            mse = mean_squared_error(self.y_val, predictions, multioutput='uniform_average')
            mse_scores[idx] = mse
        
        return mse_scores
        
        
    def optimize_gbdt_with_pso(self):
        # Use PSO to optimize GBDT hyperparameters
        best_cost, best_pos = self.pso.optimize(self.fitness_function, iters=self.params['pso_params']['iters'],verbose = True)
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
        best_gbdt = GradientBoostingRegressor(**best_params)
        self.GBDT_model = MultiOutputRegressor(best_gbdt)
        self.GBDT_model.fit(self.X_train,self.y_train)

    def DataClean(self):
        self.X = []
        self.Y = []

    def triger_KGP(self):
        if not self.triggered:
            print("KGP Processing, Sample Size:"+str(len(self.X)))
            self.train_test_split()
            best_cost, best_pos = self.optimize_gbdt_with_pso()
            self.update_gbdt_params(best_pos)
            self.GBDT_model.fit(self.X_train,self.y_train)
            self.triggered = True
        else:
            self.Model_prediction_count +=1
            if self.Model_prediction_count == 300000:
                self.DataClean()
                self.Model_prediction_count = 0
