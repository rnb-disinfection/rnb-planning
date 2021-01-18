if rtt==nil or rtt.getTC==nil then
    error("module output_utils_rtt.lua should be called from rttlua-... and not from plain lua",0)
end


function slist( stringtable )
    local v = rtt.Variable("strings")
    v:fromtab(stringtable)
    return v
end

function report_all_etaslvar( etasl_component, reporter_component )
    vL = etasl_component:list_etasl_outputs_scalar()
    for i =0, vL["size"] do
        local name = string.match(vL[i],"%.([^%.]*)$")
        if name ~= nil then
            print(name)
            if string.sub(name,1,1) =="_" then 
                print("adding port and reporting for " ..name)
                etasl_component:add_etaslvar_outputport(name,"", slist{name})
                reporter_component:reportPort(etasl_component:getName(),name)
            end
        end
    end
    vL = etasl_component:list_etasl_outputs_vector()
    for i =0, vL["size"] do
        local name = string.match(vL[i],"%.([^%.]*)$")
        if name ~= nil then
            if string.sub(name,1,1) =="_" then 
                print("adding port and reporting for " ..name)
                etasl_component:add_etaslvar_vector_outputport(name,"", vL[i])
                reporter_component:reportPort(etasl_component:getName(),name)
            end
        end
    end
    vL = etasl_component:list_etasl_outputs_rotation()
    for i =0, vL["size"] do
        local name = string.match(vL[i],"%.([^%.]*)$")
        if name ~= nil then
            if string.sub(name,1,1) =="_" then 
                print("adding port and reporting for " ..name)
                etasl_component:add_etaslvar_rotation_outputport(name,"", vL[i])
                reporter_component:reportPort(etasl_component:getName(),name)
            end
        end
    end
    vL = etasl_component:list_etasl_outputs_frame()
    for i =0, vL["size"] do
        local name = string.match(vL[i],"%.([^%.]*)$")
        if name ~= nil then
            if string.sub(name,1,1) =="_" then 
                print("adding port and reporting for " ..name)
                etasl_component:add_etaslvar_frame_outputport(name,"", vL[i])
                reporter_component:reportPort(etasl_component:getName(),name)
            end
        end
    end
end


