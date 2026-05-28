from pyomo.environ import *


def construir_modelo(data):
    model = ConcreteModel()

    # =========================
    # CONJUNTOS
    # =========================
    model.I = Set(initialize=data['I'])
    model.J = Set(initialize=data['J'])
    model.K = Set(initialize=data['K'])

    # =========================
    # PARAMETROS
    # =========================

    # Cantidad disponible inbound
    model.supply = Param(
        model.I,
        model.K,
        initialize=data['supply'],
        default=0
    )

    # Demanda outbound
    model.demand = Param(
        model.J,
        model.K,
        initialize=data['demand'],
        default=0
    )

    # Tiempo descarga inbound
    model.unload_time = Param(
        model.I,
        initialize=data['unload_time']
    )

    # Tiempo carga outbound
    model.load_time = Param(
        model.J,
        initialize=data['load_time']
    )

    # Big M
    M = data.get('bigM', 10000)
    model.M = Param(initialize=M)

    # =========================
    # VARIABLES
    # =========================

    # Tiempos inbound
    model.A = Var(model.I, domain=NonNegativeReals)
    model.B = Var(model.I, domain=NonNegativeReals)

    # Tiempos outbound
    model.C = Var(model.J, domain=NonNegativeReals)
    model.D = Var(model.J, domain=NonNegativeReals)

    # Makespan
    model.T = Var(domain=NonNegativeReals)

    # Prioridades inbound
    model.U = Var(model.I, model.I, domain=Binary)

    # Prioridades outbound
    model.V = Var(model.J, model.J, domain=Binary)

    # Transferencia directa
    model.Y = Var(model.I, model.J, model.K, domain=Binary)

    # Relación lógica
    model.Z = Var(model.I, model.J, domain=Binary)

    # Flujo
    model.X = Var(model.I, model.J, model.K, domain=NonNegativeIntegers)

    # =========================
    # FUNCION OBJETIVO
    # =========================

    model.obj = Objective(
        expr=model.T,
        sense=minimize
    )

    # =========================
    # RESTRICCIONES
    # =========================

    # 1. Makespan
    def makespan_rule(model, j):
        return model.T >= model.D[j]

    model.makespan_cons = Constraint(model.J, rule=makespan_rule)

    # 2. Conservacion flujo inbound
    def inbound_flow_rule(model, i, k):
        return sum(model.X[i, j, k] for j in model.J) <= model.supply[i, k]

    model.inbound_flow = Constraint(model.I, model.K, rule=inbound_flow_rule)

    # 3. Conservacion flujo outbound
    def outbound_flow_rule(model, j, k):
        return sum(model.X[i, j, k] for i in model.I) >= model.demand[j, k]

    model.outbound_flow = Constraint(model.J, model.K, rule=outbound_flow_rule)

    # 4. Relacion entre flujo y transferencia
    def relation_flow_transfer_rule(model, i, j, k):
        return model.X[i, j, k] <= model.M * model.Z[i, j]

    model.relation_flow_transfer = Constraint(
        model.I,
        model.J,
        model.K,
        rule=relation_flow_transfer_rule
    )

    # 5. Tiempo descarga inbound
    def inbound_time_rule(model, i):
        return model.B[i] == model.A[i] + model.unload_time[i]

    model.inbound_time = Constraint(model.I, rule=inbound_time_rule)

    # 6. Secuencia inbound
    def inbound_sequence_rule(model, i, ip):
        if i == ip:
            return Constraint.Skip

        return model.A[ip] >= model.B[i] - model.M * (1 - model.U[i, ip])

    model.inbound_sequence = Constraint(
        model.I,
        model.I,
        rule=inbound_sequence_rule
    )

    # 7. Restriccion inversa inbound
    def inbound_reverse_rule(model, i, ip):
        if i == ip:
            return Constraint.Skip

        return model.A[i] >= model.B[ip] - model.M * model.U[i, ip]

    model.inbound_reverse = Constraint(
        model.I,
        model.I,
        rule=inbound_reverse_rule
    )

    # 8. No prioridad consigo mismo inbound
    def no_self_inbound_rule(model, i):
        return model.U[i, i] == 0

    model.no_self_inbound = Constraint(
        model.I,
        rule=no_self_inbound_rule
    )

    # 9. Tiempo carga outbound
    def outbound_time_rule(model, j):
        return model.D[j] == model.C[j] + model.load_time[j]

    model.outbound_time = Constraint(model.J, rule=outbound_time_rule)

    # 10. Secuencia outbound
    def outbound_sequence_rule(model, j, jp):
        if j == jp:
            return Constraint.Skip

        return model.C[jp] >= model.D[j] - model.M * (1 - model.V[j, jp])

    model.outbound_sequence = Constraint(
        model.J,
        model.J,
        rule=outbound_sequence_rule
    )

    # 11. Restriccion inversa outbound
    def outbound_reverse_rule(model, j, jp):
        if j == jp:
            return Constraint.Skip

        return model.C[j] >= model.D[jp] - model.M * model.V[j, jp]

    model.outbound_reverse = Constraint(
        model.J,
        model.J,
        rule=outbound_reverse_rule
    )

    # 12. No prioridad consigo mismo outbound
    def no_self_outbound_rule(model, j):
        return model.V[j, j] == 0

    model.no_self_outbound = Constraint(
        model.J,
        rule=no_self_outbound_rule
    )

    # 13. Sincronizacion inbound-outbound
    def sync_rule(model, i, j, k):
        return model.C[j] >= model.B[i] - model.M * (1 - model.Z[i, j])

    model.sync = Constraint(
        model.I,
        model.J,
        model.K,
        rule=sync_rule
    )

    return model