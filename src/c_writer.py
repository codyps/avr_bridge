#! /usr/bin/env python
"""
by Cody Schafer of Rutger University.

Written with support of a research grant (R01ES014717)
from the National Institute of Environmental Health Sciences.

Software License Agreement (BSD License)

Copyright (c) 2011, Cody Schafer
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of Adam Stambler, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
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
	def __init__(self, output):
		SimpleStateC.__init__(self, output)

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
