from numpy import array, pi

class PIDcontroller:
    def __init__(self, KP, TI, TD, fs, fcut_off, output_lim) -> None:
        self.KP = KP
        self.KI = KP/TI
        self.KD = KP*TD
        self.Ts = 1.0/fs
        self.Tc = 1.0/(2.0*pi*fcut_off)
        self.coeff_a = self.Tc/(self.Tc+self.Ts)
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = array([0.0, 0.0])
        self.output = 0.0
        self.input = array([0.0, 0.0])
        self.output_lim = abs(output_lim)
        self._idx = 0

    def _limitter(self, val):
        return self.output_lim if val > self.output_lim else ( -self.output_lim if val < -self.output_lim else val )

    def PIDoperate(self, err_input):
        self.Pout = self.KP*(err_input - self.input[self._idx^1])/self.Ts
        self.Iout = self.KI*err_input
        self.Dout[self._idx] = self.KD*(err_input - 2.0*self.input[self._idx^1] + self.input[self._idx])/(self.Tc+self.Ts)/self.Ts
        + self.coeff_a*self.Dout[self._idx^1]

        self.input[self._idx] = err_input
        
        self.output = self._limitter((self.Pout + self.Iout + self.Dout[self._idx])*self.Ts + self.output)
        self._idx ^= 1
        return self.output
    
    def PIoperate(self, err_input):
        self.Pout = self.KP*(err_input - self.input[0])/self.Ts
        self.Iout = self.KI*err_input
        self.input[0] = err_input
        self.output = self._limitter((self.Pout+self.Iout)*self.Ts + self.output)
        return self.output
        
    def PIDoperate_nonfiltering(self, err_input):
        self.Pout = self.KP*(err_input - self.input[self._idx^1])/self.Ts
        self.Iout = self.KI*err_input
        self.Dout[self._idx] = self.KD*(err_input - 2.0*self.input[self._idx^1] + self.input[self._idx])/self.Ts/self.Ts

        self.input[self._idx] = err_input
        self.output = self._limitter((self.Pout + self.Iout + self.Dout[self._idx])*self.Ts + self.output)
        self._idx ^= 1
        return self.output