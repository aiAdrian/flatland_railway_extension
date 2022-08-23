import datetime
import xml.etree.ElementTree as gfg

from flatland_extensions.FlatlandGraphBuilder import FlatlandGraphBuilder


# import all flatland dependance


class FlatlandRecifeExporter:
    def __init__(self, flatland_graph_builder: FlatlandGraphBuilder, filename: str):
        self.flatland_graph_builder = flatland_graph_builder
        self.export_recife(filename=filename)

    def create_topology_parts(self, topology_parts, topology_parts_id='tp_002'):
        topology_part = gfg.SubElement(topology_parts, "topologyPart")
        topology_part.set("topoPart_Id", '{}'.format(topology_parts_id))
        gfg.SubElement(topology_part, "length").text = '{}'.format(1600)
        gfg.SubElement(topology_part, "speednormal").text = '{}'.format(200)
        gfg.SubElement(topology_part, "speedinverse").text = '{}'.format(200)
        gfg.SubElement(topology_part, "gradient").text = '{}'.format(0)
        gfg.SubElement(topology_part, "curve").text = '{}'.format(0)

    def create_topology_sequences(self, topology_sequences, topo_seq_id='ts_001', topo_ref_id='topo_ref_id'):
        topology_sequence = gfg.SubElement(topology_sequences, "topologySequence")
        topology_sequence.set("topoSeq_Id", '{}'.format(topo_seq_id))
        topo_part_List = gfg.SubElement(topology_sequence, "topoPart_List")
        topo_part_seq_elmt = gfg.SubElement(topo_part_List, "topoPartSeqElmt")
        topo_part_seq_elmt_index = 0
        topo_part_seq_elmt.set("index", '{}'.format(topo_part_seq_elmt_index))
        topo_part_ref_id = gfg.SubElement(topo_part_seq_elmt, "topoPart_RefId")
        direction = 1
        topo_part_ref_id.set("direction", '{}'.format(direction))
        topo_part_ref_id.text = '{}'.format(topo_ref_id)

    def create_track_detection_section(self, track_detection_sections, tds_id='TDS_001'):
        track_detection_section = gfg.SubElement(track_detection_sections, "trackDetectionSection")
        track_detection_section.set("TDS_Id", 'TDS_{}'.format(tds_id))
        gfg.SubElement(track_detection_section, "name").text = '{}'.format(tds_id)
        topology_parts = gfg.SubElement(track_detection_section, "topologyParts")
        topology_sequences = gfg.SubElement(track_detection_section, "topologySequences")

        # create topology
        self.create_topology_parts(topology_parts,
                                   topology_parts_id='{}'.format(tds_id))
        self.create_topology_sequences(topology_sequences,
                                       topo_seq_id='{}-Seq_'.format(tds_id),
                                       topo_ref_id='{}'.format(tds_id))

    def append_track_detection_sections(self, parent):
        track_detection_sections = gfg.SubElement(parent, "trackDetectionSections")
        nodes = self.flatland_graph_builder.get_nodes()
        for n in nodes:
            tds_id = FlatlandGraphBuilder.get_resource_id_from_node_id(n)
            self.create_track_detection_section(track_detection_sections, tds_id=tds_id)

    def create_signal(self, signals, signal_id='S_001', aspects=0, visibility_distance=400):
        track_detection_section = gfg.SubElement(signals, "signal")
        track_detection_section.set("signal_Id", '{}'.format(signal_id))
        gfg.SubElement(track_detection_section, "name").text = '{}'.format(signal_id)
        gfg.SubElement(track_detection_section, "aspects").text = '{}'.format(aspects)
        gfg.SubElement(track_detection_section, "visibilityDistance").text = '{}'.format(visibility_distance)

    def append_signals(self, parent):
        signals = gfg.SubElement(parent, "signals")
        nodes = self.flatland_graph_builder.get_nodes()
        for n in nodes:
            tds_id = FlatlandGraphBuilder.get_resource_id_from_node_id(n)
            self.create_signal(signals, signal_id='Entry_Signal_{}'.format(tds_id))
            self.create_signal(signals, signal_id='Exit_Signal_{}'.format(tds_id))

    def create_block(self, blocks, block_id='B_001', entry_signal_ref_id='S01', exit_signal_ref_id='S02'):
        block = gfg.SubElement(blocks, "block")
        block.set("block_Id", 'Block_{}'.format(block_id))
        gfg.SubElement(block, "name").text = '{}:{}:{}'.format(block_id,
                                                               entry_signal_ref_id,
                                                               exit_signal_ref_id)
        gfg.SubElement(block, "entrySignal_RefId").text = '{}'.format(entry_signal_ref_id)
        gfg.SubElement(block, "exitSignal_RefId").text = '{}'.format(exit_signal_ref_id)
        gfg.SubElement(block, "sightDistanceTDS_RefId").text = 'TDS_{}'.format(block_id)
        gfg.SubElement(block, "formationTime").text = '{}'.format(0)
        gfg.SubElement(block, "releaseTime").text = '{}'.format(0)
        tds_topology_list = gfg.SubElement(block, "TDS_Topology_List")
        tds_topology = gfg.SubElement(tds_topology_list, "TDS_Topology")
        tds_topology.set('index', '0')
        gfg.SubElement(tds_topology, "TDS_RefId").text = 'TDS_{}'.format(block_id)
        gfg.SubElement(tds_topology, "topoSeq_RefId").text = '{}-Seq_'.format(block_id)

    def append_blocks(self, parent):
        blocks = gfg.SubElement(parent, "blocks")
        nodes = self.flatland_graph_builder.get_nodes()
        for n in nodes:
            tds_id = FlatlandGraphBuilder.get_resource_id_from_node_id(n)
            self.create_block(blocks,
                              block_id='{}'.format(tds_id),
                              entry_signal_ref_id='Entry_Signal_{}'.format(tds_id),
                              exit_signal_ref_id='Exit_Signal_{}'.format(tds_id))

    def append_infrastructure_definition(self, recife_objects):
        infrastructure_definition = gfg.SubElement(recife_objects, "InfrastructureDefinition")
        infrastructure_definition.set("infrastructure_Id", "flatland")
        gfg.SubElement(infrastructure_definition, "name").text = 'flatland'
        gfg.SubElement(infrastructure_definition, "description").text = '{}'.format(datetime.datetime.now())

        self.append_track_detection_sections(infrastructure_definition)
        self.append_signals(infrastructure_definition)
        self.append_blocks(infrastructure_definition)

    def export_recife(self, filename):
        recife_objects = gfg.Element("recifeObjects")
        recife_objects.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
        recife_objects.set('xsi:noNamespaceSchemaLocation', 'all.xsd')
        self.append_infrastructure_definition(recife_objects)

        tree = gfg.ElementTree(recife_objects)
        with open(filename, "wb") as files:
            tree.write(files,
                       encoding="UTF-8",
                       xml_declaration=True)
