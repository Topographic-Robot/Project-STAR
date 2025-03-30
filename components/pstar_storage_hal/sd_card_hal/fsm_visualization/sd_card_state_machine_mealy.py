# components/pstar_storage_hal/sd_card_hal/fsm_visualization/sd_card_state_machine_mealy.py

from graphviz import Digraph
import os

# --- Function for Descriptive Diagram ---
def create_sd_card_descriptive_graph(filename="sd_card_state_machine_descriptive"):
    """
    Generates a Graphviz diagram of the SD Card HAL state machine
    using descriptive labels based on code events and conditions.

    Args:
        filename (str): The base name for the output files.
    """
    dot = Digraph(comment='SD Card HAL State Machine (Descriptive)', format='png')

    # Graph attributes for clarity
    dot.attr(rankdir='LR')
    dot.attr(fontsize='14')
    dot.attr(nodesep='0.6', ranksep='1.0')
    dot.attr(dpi='150')

    dot.attr('node', shape='box', style='rounded', fontsize='14')
    dot.attr('edge', fontsize='12')

    # Define States with descriptive labels
    states = [
        ('IDLE', 'IDLE\n(No Card/Unmounted)'),
        ('CARD_INSERTED', 'CARD_INSERTED\n(Card Detected)'),
        ('INTERFACE_DISCOVERY', 'INTERFACE_DISCOVERY\n(Trying SDIO/SPI...)'),
        ('INTERFACE_READY', 'INTERFACE_READY\n(Mounted & Usable)'),
        ('ERROR', 'ERROR\n(Mount/Health/Reset Fail)'),
        ('FAILED', 'FAILED\n(Non-recoverable Error)'),
    ]

    for name, label in states:
        dot.node(name, label)

    # Define Transitions with descriptive labels from code analysis
    transitions = [
        ('IDLE', 'CARD_INSERTED', 'Card Inserted\n(debounced ISR)'),
        ('CARD_INSERTED', 'INTERFACE_DISCOVERY', 'Auto-progress\n(in task loop)'),
        ('INTERFACE_DISCOVERY', 'INTERFACE_READY', 'Interface OK &\nMount OK\n(priv_sd_card_try_interfaces,\npriv_sd_card_mount succeed)'),
        ('INTERFACE_DISCOVERY', 'ERROR', 'Interface/Mount Failed\n(priv_sd_card_try_interfaces or\npriv_sd_card_mount fail)'),
        ('INTERFACE_DISCOVERY', 'IDLE', 'Card Removed\n(during discovery)'),
        ('INTERFACE_READY', 'IDLE', 'Card Removed\n(calls priv_sd_card_unmount)'),
        ('INTERFACE_READY', 'ERROR', 'Health Check Fail &\nReset Fail\n(stat fails & priv_sd_card_reset fails)'),
        ('INTERFACE_READY', 'CARD_INSERTED', 'Force Remount Called\n(sd_card_force_remount)'),
        ('ERROR', 'INTERFACE_READY', 'Retry Succeeded\n(priv_sd_card_reset succeeds)'),
        ('ERROR', 'FAILED', 'Retry Failed &\nMax Retries Reached\n(priv_sd_card_reset fails\n& error_handler limit)'),
        ('ERROR', 'IDLE', 'Card Removed\n(while in error state)'),
        ('FAILED', 'IDLE', 'Card Removed\n(only exit from FAILED)'),
    ]

    for source, target, label in transitions:
        dot.edge(source, target, label=label)

    # Render the graph
    try:
        output_path = dot.render(filename, view=False, cleanup=True)
        print(f"Descriptive state machine graph saved as: {output_path}")
    except Exception as e:
        print(f"Error rendering descriptive graph: {e}")
        print(f"DOT source was saved to {filename}.gv")

# --- Function for Mealy-Style Input/Output Diagram ---
def create_sd_card_mealy_style_graph(filename="sd_card_state_machine_mealy"):
    """
    Generates a Graphviz diagram of the SD Card HAL state machine
    using Mealy-style Input / Output notation derived from code behavior.

    Args:
        filename (str): The base name for the output files.
    """
    dot = Digraph(comment='SD Card HAL State Machine (Mealy Input/Output)', format='png')

    # Graph attributes
    dot.attr(rankdir='LR')
    dot.attr(fontsize='12')
    dot.attr(nodesep='0.5', ranksep='0.8')
    dot.attr(dpi='150')

    dot.attr('node', shape='circle', fontsize='12') # Classic circle shape
    dot.attr('edge', fontsize='10')

    # Define States (using shorter names for nodes)
    states = [
        ('IDLE', 'IDLE'),
        ('INSERTED', 'CARD_INSERTED'),
        ('DISCOVERY', 'INTERFACE_DISCOVERY'),
        ('READY', 'INTERFACE_READY'),
        ('ERROR', 'ERROR'),
        ('FAILED', 'FAILED'),
    ]

    for name, _ in states:
         dot.node(name, name) # Use simple name in circle

    # Define Transitions with Input / Output notation derived from code actions
    # Input: Event or Condition triggering the transition
    # Output: Primary action performed during the transition
    transitions = [
        # Input                     # Output (Action during transition)
        ('IDLE', 'INSERTED',        'Card Inserted ISR / -'),
        ('INSERTED', 'DISCOVERY',   'Task Loop Progress / Initiate Discovery+Mount'),
        ('DISCOVERY', 'READY',      'Discovery+Mount OK / Save Config'),
        ('DISCOVERY', 'ERROR',      'Discovery or Mount Fail / Log Error'),
        ('DISCOVERY', 'IDLE',       'Card Removed / -'),
        ('READY', 'IDLE',           'Card Removed / Perform Unmount'),
        ('READY', 'ERROR',          'Health Check Fail + Reset Fail / Log Error'),
        ('READY', 'INSERTED',       'Force Remount Call / Perform Unmount'),
        ('ERROR', 'READY',          'Reset OK (Retry) / -'),
        ('ERROR', 'FAILED',         'Reset Fail (Max Retries) / Log Error'),
        ('ERROR', 'IDLE',           'Card Removed / -'),
        ('FAILED', 'IDLE',          'Card Removed / -'),
    ]

    for source, target, label in transitions:
        dot.edge(source, target, label=label)

    # Render the graph
    try:
        output_path = dot.render(filename, view=False, cleanup=True)
        print(f"Mealy-style state machine graph saved as: {output_path}")
        print("\n--- Mealy Diagram Key ---")
        print("This diagram uses 'Input / Output' notation on transitions:")
        print(" 'Input': Represents the event or condition from the code causing the transition.")
        print("   - Card Inserted ISR: Debounced signal from card detect pin.")
        print("   - Task Loop Progress: Automatic step within the mount task.")
        print("   - Discovery+Mount OK/Fail: Result of priv_sd_card_try_interfaces/priv_sd_card_mount.")
        print("   - Card Removed: Debounced signal from card detect pin.")
        print("   - Health Check Fail + Reset Fail: Result of stat() and subsequent priv_sd_card_reset.")
        print("   - Force Remount Call: sd_card_force_remount() function invoked.")
        print("   - Reset OK/Fail (Retry): Result of priv_sd_card_reset during error recovery.")
        print(" 'Output': Represents the primary action performed by the code *during* that transition.")
        print("   - Initiate Discovery+Mount: Calling priv_sd_card_try_interfaces.")
        print("   - Save Config: Calling priv_save_working_config.")
        print("   - Log Error: Error logging action (e.g., via log_error, RECORD_ERROR).")
        print("   - Perform Unmount: Calling priv_sd_card_unmount.")
        print("   - '-': No distinct major action, primarily internal state change or simple logging.")
        print("--------------------------")

    except Exception as e:
        print(f"Error rendering Mealy-style graph: {e}")
        print(f"DOT source was saved to {filename}.gv")


if __name__ == "__main__":
    print("Generating Descriptive State Machine Diagram...")
    create_sd_card_descriptive_graph()
    print("\nGenerating Mealy-Style Input/Output Diagram...")
    create_sd_card_mealy_style_graph()
    print("\nDone.")
