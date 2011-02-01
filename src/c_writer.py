# vim: sw=8 ts=8 sts=8 noet 

class SimpleStateC():
	def __init__(self, output):
		self.out = output
		self.b_space = '\t'
		self.m_space = ' '
		self.b_indent = 0
		self.m_indent = 0
	
	def _w(self, rtxt):
		""" write out raw text to output """
		self.out.write(rtxt)

	def _wi(self):
		""" write out the current indent """
		self._w(self.b_space * self.b_indent)

	def macro_indent(self):
		self.m_indent += 1

	def macro_dedent(self):
		self.m_indent -= 1

	def indent(self):
		self.b_indent += 1

	def dedent(self):
		self.b_indent -= 1
	
	def line(self, txt):
		self._wi()
		self._w(txt)
		self._w('\n')
	
	def macro_line(self, mline):
		self._w('#')
		self._w(self.m_space * self.m_indent)
		self._w(mline)
		self._w('\n')

	def close(self):
		self.out.close()

class CStateTracker(SimpleStateC):
	def __init__(output):
		self.out = output
		self.b_space = '\t'
		self.m_space = ' '
		self.b_indent = 0 # normal (brace) indent
		self.m_indent = 0 # macro indent

	def _do_brace(self, brace_on_line=True):
		"""write out a brace, update indent.
		   also advances to a new line.
		"""
		if brace_on_line:
			self._w(' {')
		else:
			self._w('\n')
			self._wi()
			self._w('{')

		self._w('\n')
		self.indent()

	def tagged_block(self, tag, brace_on_line=True):
		self._wi()
		self._w(tag)
		self._do_brace(brace_on_line)

	def end_block(self, cuddled_text='', new_block=None):
		self._wi()
		self._w('}')

	def line(self, rline):
		""" write a normal line out """
		self._wi()
		self._w(rline)
		self._w('\n')

	def macro_line(self, rline):
		""" write a macro line out
		    macros have different indent rules
		"""
		self._w('#')
		self._w(self.m_space * self.m_indent)
		self._w(rline)
		self._w('\n')

	def _m_down(self, txt):
		self.macro_dedent()
		self.macro_line(self, txt)

	def _m_both(self, txt):
		self._m_down(txt)
		self.macro_indent()

	def _m_up(self, txt):
		self.macro_line(txt)
		self.macro_indent()

	def macro_if(self, condition):
		self._m_up('if {0}'.format(condition))

	def macro_ifdef(self, symbol):
		self._m_up('ifdef {0}'.format(condition))
		
	def macro_else(self):
		self._m_both('else')

	def macro_elif(self, condition):
		self.m_both('elif {0}'.format(condition))

	def macro_endif(self):
		self._m_down('endif')
