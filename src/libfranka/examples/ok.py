import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-5, 5, 400)

# Fonctions
f1 = 2 * x + 1
f2 = x**2 - 3 * x + 2
f3 = x**5 - 4 * x**3 + x

# Dérivées premières (approximation)
df1 = np.gradient(f1, x)
df2 = np.gradient(f2, x)
df3 = np.gradient(f3, x)

# Dérivées secondes (approximation)
ddf1 = np.gradient(df1, x)
ddf2 = np.gradient(df2, x)
ddf3 = np.gradient(df3, x)

# Création des sous-graphes
fig, axs = plt.subplots(3, 1, figsize=(10, 9))

# Fonction affine
axs[0].plot(x, f1, label="f(x)")
axs[0].plot(x, df1, label="f'(x)")
axs[0].plot(x, ddf1, label="f''(x)")
axs[0].set_title("Fonction affine : f(x) = 2x + 1")
axs[0].legend()
axs[0].grid(True)

# Fonction quadratique
axs[1].plot(x, f2, label="f(x)")
axs[1].plot(x, df2, label="f'(x)")
axs[1].plot(x, ddf2, label="f''(x)")
axs[1].set_title("Fonction quadratique : f(x) = x² - 3x + 2")
axs[1].legend()
axs[1].grid(True)

# Fonction degré 5
axs[2].plot(x, f3, label="f(x)")
axs[2].plot(x, df3, label="f'(x)")
axs[2].plot(x, ddf3, label="f''(x)")
axs[2].set_title("Fonction de degré 5 : f(x) = x⁵ - 4x³ + x")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()
