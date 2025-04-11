#!/usr/bin/env python3

import streamlit as st
from typing import Dict, Tuple, Optional, List
import numpy as np
from dataclasses import dataclass
from pathlib import Path
class SubmapAnalyzerState:
    @staticmethod
    def initialize():
        if 'initialized' not in st.session_state:
            st.session_state.working_path = None

    @staticmethod
    def set_working_path(working_path: Path):
        st.session_state.working_path = Path(working_path)

    @staticmethod
    def get_working_path() -> Path:
        return st.session_state.working_path