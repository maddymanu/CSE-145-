class PID:
	
	def __init__(self, P=1, I=1, D=8, der=0, integ=0, int_max=700, int_min=-700):

		self.k_p_val=P
		self.k_i_val=I
		self.k_d_val=D

		self.der=der
		self.integ =integ

		self.int_max=int_max
		self.int_min=int_min

		self.point=0.0
		self.er=0.0

	

	def get_new_val(self,inp):
		

		self.err = self.point - inp

		self.P__NEW_VALUE = self.k_p_val * self.err
		self.D_NEW_VALUE = self.K_d_val * ( self.err - self.der)
		self.der = self.err

		self.integ = self.integ + self.err

		if self.integ > self.int_max:
			self.integ = self.int_max
		elif self.integ < self.int_min:
			self.integ = self.int_min

		self.I_NEW_VALUE = self.integ * self.K_i_val

		val = self.P_NEW_VALUE + self.I_NEW_VALUE + self.D_NEW_VALUE

		return val

	def setPoint(self,inp):
		
		self.point = inp
		self.integ=0
		self.der=0

