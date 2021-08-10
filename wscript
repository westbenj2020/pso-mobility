## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('pso-mobility', ['core'])
    module.source = [
        'model/pso-mobility.cc'
        ]
    
    headers = bld(features='ns3header')
    headers.module = 'pso-mobility'
    headers.source = [
        'model/pso-mobility.h'
        ]
    
    if (bld.env['ENABLE_EXAMPLES']):
        bld.recurse('examples')

    bld.ns3_python_bindings()
