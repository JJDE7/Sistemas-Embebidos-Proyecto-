class PIDController:
    def __init__(self, kp):
        """
        Inicializa el controlador PID.
        :param kp: Constante de ganancia proporcional.
        """
        self.kp = kp

    def calcular_ajuste(self, error):
        """
        Calcula el ajuste proporcional basado en el error.
        :param error: Diferencia entre el valor objetivo y el valor actual.
        :return: Ajuste calculado.
        """
        ajuste = self.kp * error
        return ajuste